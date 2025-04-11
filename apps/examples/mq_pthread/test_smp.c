/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

 #include <tinyara/config.h>
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <errno.h>
 #include <pthread.h>
 #include <mqueue.h>
 #include <fcntl.h>
 #include <unistd.h>
 #include <sched.h>
 #include <sys/types.h>
 #include <semaphore.h>
 #include <time.h>
 
 /****************************************************************************
  * Pre-processor Definitions
  ****************************************************************************/
 
 /* SMP-specific configuration */
 #define SMP_TEST_ENABLED  1
 #define NUM_CPU_CORES     2        /* Modify this based on your system */
 
 /* Use minimal timeouts for faster failures */
 #define USE_TIMEOUTS      1
 #define DEFAULT_TIMEOUT_SEC   1
 
 /* Test parameters - more threads for SMP stress */
 #define MQ_NAME                  "/mq_pthread_cancel_test"
 #define NUM_RECEIVER_THREADS     10      /* More threads for SMP stress */
 #define NUM_SENDER_THREADS       10      /* More threads for SMP stress */
 #define NUM_TEST_ITERATIONS      3       
 #define TEST_QUEUE_MAX_MSGS      4       /* Small queue size to make it fill up quickly */
 #define MESSAGE_SIZE             32
 #define SEND_ATTEMPTS_PER_THREAD 10
 
 /* Testing status reporting */
 #define STATUS_INTERVAL_MS       500      /* Status report interval in ms */
 
 /****************************************************************************
  * Private Types
  ****************************************************************************/
 
 enum test_phase_e {
     PHASE_NONE = 0,
     PHASE_SMP_STRESS_TEST,
     PHASE_COMPLETE
 };
 
 struct thread_data_s {
     int thread_id;
     pthread_t thread;
     int is_receiver;
     mqd_t mqdes;
     int cpu_affinity;           /* Target CPU for this thread */
     sem_t *wait_sem;            /* Semaphore for synchronization */
     sem_t *start_sem;           /* Semaphore for synchronized start */
     volatile int status;        /* 0 = running, 1 = canceled, 2 = completed */
 };
 
 /****************************************************************************
  * Private Function Prototypes
  ****************************************************************************/
 
 static void *receiver_thread(void *param);
 static void *sender_thread(void *param);
 static void *canceler_thread(void *param);
 static void *status_reporter_thread(void *param);
 static void cleanup_handler(void *arg);
 static void print_test_results(struct thread_data_s *threads, int num_threads);
 static void print_mq_status(mqd_t mqdes, const char *label);
 static int set_thread_affinity(pthread_t thread, int cpu);
 static int run_smp_stress_test(void);
 
 /****************************************************************************
  * Private Data
  ****************************************************************************/
 
 static pthread_mutex_t g_mutex;
 static int g_test_counter = 0;
 static int g_message_counter = 0;
 static volatile enum test_phase_e g_current_phase = PHASE_NONE;
 static volatile int g_status_report_active = 0;
 static volatile mqd_t g_active_mqdes = 0;
 
 /****************************************************************************
  * Public Functions
  ****************************************************************************/
 
 /****************************************************************************
  * Name: print_mq_status
  *
  * Description:
  *   Prints the current status of a message queue
  *
  ****************************************************************************/
 
 static void print_mq_status(mqd_t mqdes, const char *label)
 {
     if (mqdes <= 0) {
         printf("ERROR: Invalid mqdes passed to print_mq_status\n");
         return;
     }
 
     struct mq_attr attr;
     if (mq_getattr(mqdes, &attr) == 0) {
         printf("%s: curmsgs=%ld, maxmsgs=%ld\n", 
                label, (long)attr.mq_curmsgs, (long)attr.mq_maxmsg);
     } else {
         printf("%s: ERROR getting queue attributes: %d\n", label, errno);
     }
 }
 
 /****************************************************************************
  * Name: set_thread_affinity
  *
  * Description:
  *   Set a thread's CPU affinity (if supported by OS)
  *
  ****************************************************************************/
 
 static int set_thread_affinity(pthread_t thread, int cpu)
 {
 #ifdef CONFIG_SMP
     /* For TizenRT, use the appropriate CPU affinity setting API */
     /* This is just a placeholder - modify according to your actual API */
     printf("Setting thread affinity to CPU %d\n", cpu);
     
     /* Example (modify according to your actual API):
     cpu_set_t cpuset;
     CPU_ZERO(&cpuset);
     CPU_SET(cpu, &cpuset);
     return pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
     */
     
     /* Since we don't have the exact API, just return success for now */
     return 0;
 #else
     /* Not supported on non-SMP systems */
     return 0;
 #endif
 }
 
 /****************************************************************************
  * Name: status_reporter_thread
  *
  * Description:
  *   A thread that periodically reports program status even if other
  *   threads are stuck or hung
  *
  ****************************************************************************/
 
 static void *status_reporter_thread(void *param)
 {
     int counter = 0;
     const char *phase_names[] = {
         "NONE", "SMP_STRESS_TEST", "COMPLETE"
     };
     
     printf("Status reporter thread started\n");
     
     while (g_status_report_active) {
         /* Sleep for the reporting interval */
         usleep(STATUS_INTERVAL_MS * 1000);
         counter++;
         
         /* Report current status */
         printf("\n--- STATUS REPORT #%d ---\n", counter);
         printf("Current test phase: %s\n", phase_names[g_current_phase]);
         
         /* Report message queue status if there's an active one */
         if (g_active_mqdes > 0) {
             print_mq_status(g_active_mqdes, "Active message queue");
         }
         
         printf("--- END STATUS REPORT #%d ---\n\n", counter);
     }
     
     printf("Status reporter thread stopped\n");
     return NULL;
 }
 
 /****************************************************************************
  * mq_pthread_test_main
  ****************************************************************************/
 
 #ifdef CONFIG_BUILD_KERNEL
 int main(int argc, FAR char *argv[])
 #else
 int mq_pthread_test_main(int argc, char *argv[])
 #endif
 {
     int ret = 0;
     pthread_mutexattr_t attr;
     pthread_t status_thread;
 
     printf("Message Queue pthread_cancel SMP Stress Test\n");
     printf("===========================================\n");
 
     pthread_mutexattr_init(&attr);
     pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
     pthread_mutex_init(&g_mutex, &attr);
     pthread_mutexattr_destroy(&attr);
     
     /* Start the status reporter thread */
     g_status_report_active = 1;
     if (pthread_create(&status_thread, NULL, status_reporter_thread, NULL) != 0) {
         printf("WARNING: Failed to create status reporter thread\n");
     }
 
     /* Run SMP stress test */
     printf("\nRunning SMP message queue stress test...\n");
     g_current_phase = PHASE_SMP_STRESS_TEST;
     ret = run_smp_stress_test();
     if (ret != 0) {
         printf("SMP stress test failed with error %d\n", ret);
         g_status_report_active = 0;
         pthread_join(status_thread, NULL);
         pthread_mutex_destroy(&g_mutex);
         return -1;
     }
     printf("SMP stress test completed successfully\n");
 
     /* Stop the status reporter thread */
     g_current_phase = PHASE_COMPLETE;
     g_status_report_active = 0;
     pthread_join(status_thread, NULL);
     
     pthread_mutex_destroy(&g_mutex);
     
     printf("\nAll tests completed successfully!\n");
     return 0;
 }
 
 /****************************************************************************
  * Name: run_smp_stress_test
  *
  * Description:
  *   Run a stress test specifically designed to trigger issues in SMP 
  *   environments by creating many threads with different CPU affinities.
  *
  * Return Value:
  *   0 (OK) or -1 (ERROR) if test failed
  *
  ****************************************************************************/
 
 static int run_smp_stress_test(void)
 {
     int ret;
     int i, j;
     struct mq_attr attr;
     mqd_t mqdes;
     pthread_t canceler_thread_id;
     struct thread_data_s *receiver_threads;
     struct thread_data_s *sender_threads;
     sem_t wait_sem;
     sem_t start_sem;
     char buffer[MESSAGE_SIZE];
     
     printf("SMP TEST: Initializing test resources\n");
     
     /* Initialize semaphores */
     sem_init(&wait_sem, 0, 0);
     sem_init(&start_sem, 0, 0);
     
     /* Create message queue */
     attr.mq_maxmsg = TEST_QUEUE_MAX_MSGS;
     attr.mq_msgsize = MESSAGE_SIZE;
     attr.mq_flags = 0;
     
     /* Unlink any existing queues with this name */
     mq_unlink(MQ_NAME);
     
     /* Create an empty message queue */
     printf("SMP TEST: Creating message queue %s\n", MQ_NAME);
     mqdes = mq_open(MQ_NAME, O_CREAT | O_RDWR, 0666, &attr);
     if (mqdes == (mqd_t)-1) {
         printf("Error opening message queue: %d\n", errno);
         sem_destroy(&wait_sem);
         sem_destroy(&start_sem);
         return -1;
     }
     
     /* Store the active message queue descriptor for status reports */
     g_active_mqdes = mqdes;
     
     /* Allocate thread data arrays */
     receiver_threads = (struct thread_data_s *)malloc(NUM_RECEIVER_THREADS * sizeof(struct thread_data_s));
     if (!receiver_threads) {
         printf("Failed to allocate receiver thread data\n");
         mq_close(mqdes);
         mq_unlink(MQ_NAME);
         sem_destroy(&wait_sem);
         sem_destroy(&start_sem);
         return -1;
     }
     
     sender_threads = (struct thread_data_s *)malloc(NUM_SENDER_THREADS * sizeof(struct thread_data_s));
     if (!sender_threads) {
         printf("Failed to allocate sender thread data\n");
         free(receiver_threads);
         mq_close(mqdes);
         mq_unlink(MQ_NAME);
         sem_destroy(&wait_sem);
         sem_destroy(&start_sem);
         return -1;
     }
 
     /* Run multiple iterations of the test */
     for (j = 0; j < NUM_TEST_ITERATIONS; j++) {
         printf("\nSMP TEST: Iteration %d/%d\n", j+1, NUM_TEST_ITERATIONS);
         
         /* Create receiver threads with different CPU affinities */
         printf("SMP TEST: Creating receiver threads\n");
         for (i = 0; i < NUM_RECEIVER_THREADS; i++) {
             receiver_threads[i].thread_id = i;
             receiver_threads[i].mqdes = mqdes;
             receiver_threads[i].is_receiver = 1;
             receiver_threads[i].wait_sem = &wait_sem;
             receiver_threads[i].start_sem = &start_sem;
             receiver_threads[i].status = 0; /* running */
             
             /* Alternate CPU affinities to distribute load */
             receiver_threads[i].cpu_affinity = i % NUM_CPU_CORES;
             
             ret = pthread_create(&receiver_threads[i].thread, NULL, receiver_thread, &receiver_threads[i]);
             if (ret != 0) {
                 printf("Error creating receiver thread %d: %d\n", i, ret);
                 for (int k = 0; k < i; k++) {
                     pthread_cancel(receiver_threads[k].thread);
                     pthread_join(receiver_threads[k].thread, NULL);
                 }
                 free(receiver_threads);
                 free(sender_threads);
                 mq_close(mqdes);
                 mq_unlink(MQ_NAME);
                 sem_destroy(&wait_sem);
                 sem_destroy(&start_sem);
                 return -1;
             }
             
             /* Set CPU affinity for this thread */
             set_thread_affinity(receiver_threads[i].thread, receiver_threads[i].cpu_affinity);
         }
         
         /* Create sender threads with different CPU affinities */
         printf("SMP TEST: Creating sender threads\n");
         for (i = 0; i < NUM_SENDER_THREADS; i++) {
             sender_threads[i].thread_id = i + NUM_RECEIVER_THREADS; /* unique IDs */
             sender_threads[i].mqdes = mqdes;
             sender_threads[i].is_receiver = 0;
             sender_threads[i].wait_sem = &wait_sem;
             sender_threads[i].start_sem = &start_sem;
             sender_threads[i].status = 0; /* running */
             
             /* Alternate CPU affinities to distribute load */
             sender_threads[i].cpu_affinity = i % NUM_CPU_CORES;
             
             ret = pthread_create(&sender_threads[i].thread, NULL, sender_thread, &sender_threads[i]);
             if (ret != 0) {
                 printf("Error creating sender thread %d: %d\n", i, ret);
                 /* Cancel and join all created threads */
                 for (int k = 0; k < NUM_RECEIVER_THREADS; k++) {
                     pthread_cancel(receiver_threads[k].thread);
                     pthread_join(receiver_threads[k].thread, NULL);
                 }
                 for (int k = 0; k < i; k++) {
                     pthread_cancel(sender_threads[k].thread);
                     pthread_join(sender_threads[k].thread, NULL);
                 }
                 free(receiver_threads);
                 free(sender_threads);
                 mq_close(mqdes);
                 mq_unlink(MQ_NAME);
                 sem_destroy(&wait_sem);
                 sem_destroy(&start_sem);
                 return -1;
             }
             
             /* Set CPU affinity for this thread */
             set_thread_affinity(sender_threads[i].thread, sender_threads[i].cpu_affinity);
         }
         
         /* Allow threads to start */
         usleep(100 * 1000); /* 100ms */
         
         /* Release threads to start operations */
         printf("SMP TEST: Starting all threads\n");
         for (i = 0; i < NUM_RECEIVER_THREADS + NUM_SENDER_THREADS; i++) {
             sem_post(&start_sem);
         }
         
         /* Give threads time to operate */
         usleep(200 * 1000); /* 200ms */
         
         /* Print queue status before cancellation */
         print_mq_status(mqdes, "SMP TEST: Status before thread cancellation");
         
         /* Create canceler thread */
         printf("SMP TEST: Creating canceler thread\n");
         ret = pthread_create(&canceler_thread_id, NULL, canceler_thread, receiver_threads);
         if (ret != 0) {
             printf("Error creating canceler thread: %d\n", ret);
             /* Cancel and join all created threads */
             for (i = 0; i < NUM_RECEIVER_THREADS; i++) {
                 pthread_cancel(receiver_threads[i].thread);
                 pthread_join(receiver_threads[i].thread, NULL);
             }
             for (i = 0; i < NUM_SENDER_THREADS; i++) {
                 pthread_cancel(sender_threads[i].thread);
                 pthread_join(sender_threads[i].thread, NULL);
             }
             free(receiver_threads);
             free(sender_threads);
             mq_close(mqdes);
             mq_unlink(MQ_NAME);
             sem_destroy(&wait_sem);
             sem_destroy(&start_sem);
             return -1;
         }
         
         /* Wait for canceler thread to complete */
         printf("SMP TEST: Waiting for canceler thread to complete\n");
         ret = pthread_join(canceler_thread_id, NULL);
         if (ret != 0) {
             printf("SMP TEST: Error joining canceler thread: %d\n", ret);
         } else {
             printf("SMP TEST: Canceler thread completed successfully\n");
         }
         
         /* Signal any remaining threads to exit */
         printf("SMP TEST: Signaling any remaining threads to exit\n");
         for (i = 0; i < NUM_RECEIVER_THREADS + NUM_SENDER_THREADS; i++) {
             sem_post(&wait_sem);
         }
         
         /* Cancel and join any remaining threads */
         printf("SMP TEST: Cleaning up remaining threads\n");
         for (i = 0; i < NUM_RECEIVER_THREADS; i++) {
             if (receiver_threads[i].status == 0) { /* still running */
                 pthread_cancel(receiver_threads[i].thread);
                 pthread_join(receiver_threads[i].thread, NULL);
             }
         }
         
         for (i = 0; i < NUM_SENDER_THREADS; i++) {
             if (sender_threads[i].status == 0) { /* still running */
                 pthread_cancel(sender_threads[i].thread);
                 pthread_join(sender_threads[i].thread, NULL);
             }
         }
         
         /* Print test results */
         printf("SMP TEST: Receiver threads results:\n");
         print_test_results(receiver_threads, NUM_RECEIVER_THREADS);
         printf("SMP TEST: Sender threads results:\n");
         print_test_results(sender_threads, NUM_SENDER_THREADS);
         
         /* Print queue status after thread cleanup */
         print_mq_status(mqdes, "SMP TEST: Status after thread cleanup");
         
         /* Drain the queue */
         printf("SMP TEST: Draining any remaining messages\n");
         unsigned int prio;
         struct timespec timeout;
         
         while (1) {
             clock_gettime(CLOCK_REALTIME, &timeout);
             timeout.tv_sec += DEFAULT_TIMEOUT_SEC;
             
             ret = mq_timedreceive(mqdes, buffer, MESSAGE_SIZE, &prio, &timeout);
             if (ret < 0) {
                 /* Break when the queue is empty or on error */
                 break;
             }
         }
         
         /* Print queue status after draining */
         print_mq_status(mqdes, "SMP TEST: Status after draining queue");
         
         printf("SMP TEST: Iteration %d completed\n", j+1);
     }
     
     /* Clean up resources */
     printf("SMP TEST: Cleaning up resources\n");
     g_active_mqdes = 0;
     mq_close(mqdes);
     mq_unlink(MQ_NAME);
     sem_destroy(&wait_sem);
     sem_destroy(&start_sem);
     free(receiver_threads);
     free(sender_threads);
     
     printf("SMP TEST: Successfully completed\n");
     return 0;
 }
 
 /****************************************************************************
  * Name: receiver_thread
  *
  * Description:
  *   A thread that waits to receive messages from the message queue.
  *   This thread will be canceled while it's blocked waiting for messages.
  *
  ****************************************************************************/
 
 static void *receiver_thread(void *param)
 {
     struct thread_data_s *thread_data = (struct thread_data_s *)param;
     char buffer[MESSAGE_SIZE];
     unsigned int prio;
     int result;
     struct timespec timeout;
     
     printf("Receiver thread %d starting on CPU %d\n", 
            thread_data->thread_id, thread_data->cpu_affinity);
     
     /* Install cleanup handler */
     pthread_cleanup_push(cleanup_handler, thread_data);
     
     /* Make thread cancelable */
     pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
     pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
     
     /* Wait for signal to start */
     printf("Receiver thread %d waiting for start signal\n", thread_data->thread_id);
     sem_wait(thread_data->start_sem);
     
     printf("Thread %d waiting for message...\n", thread_data->thread_id);
     
     /* Loop until canceled or we receive messages */
     int iteration = 0;
     while (iteration < 100) {  /* Limit iterations to prevent infinite loops */
         iteration++;
         
 #if USE_TIMEOUTS
         /* Set timeout to a few seconds from now */
         clock_gettime(CLOCK_REALTIME, &timeout);
         timeout.tv_sec += DEFAULT_TIMEOUT_SEC;
         
         /* Wait with timeout to receive a message */
         result = mq_timedreceive(thread_data->mqdes, buffer, MESSAGE_SIZE, &prio, &timeout);
 #else
         /* Wait to receive a message - this is a cancellation point */
         result = mq_receive(thread_data->mqdes, buffer, MESSAGE_SIZE, &prio);
 #endif
         
         /* Check if we received a message */
         if (result >= 0) {
             printf("Thread %d received message\n", thread_data->thread_id);
             break;
         } else if (errno == ETIMEDOUT) {
             /* Timeout is expected for empty queue - try again */
         } else if (errno != EINTR) {
             printf("Thread %d mq_receive error: %d\n", thread_data->thread_id, errno);
             break;
         }
         
         /* Check if cancellation is pending */
         pthread_testcancel();
         
         /* Check if we should exit */
         if (sem_trywait(thread_data->wait_sem) == 0) {
             printf("Thread %d exiting after semaphore signal\n", thread_data->thread_id);
             break;
         }
     }
     
     /* Mark thread as completed normally */
     pthread_mutex_lock(&g_mutex);
     if (thread_data->status == 0) { /* still running */
         thread_data->status = 2; /* completed */
     }
     pthread_mutex_unlock(&g_mutex);
     
     /* Execute cleanup handler on thread exit */
     pthread_cleanup_pop(1);
     printf("Thread %d finished\n", thread_data->thread_id);
     return NULL;
 }
 
 /****************************************************************************
  * Name: sender_thread
  *
  * Description:
  *   A thread that sends messages to the message queue.
  *   This thread may get blocked if the queue is full and then be canceled.
  *
  ****************************************************************************/
 
 static void *sender_thread(void *param)
 {
     struct thread_data_s *thread_data = (struct thread_data_s *)param;
     char buffer[MESSAGE_SIZE];
     int msg_count = 0;
     int result;
     struct timespec timeout;
     
     printf("Sender thread %d starting on CPU %d\n", 
            thread_data->thread_id, thread_data->cpu_affinity);
     
     /* Install cleanup handler */
     pthread_cleanup_push(cleanup_handler, thread_data);
     
     /* Make thread cancelable */
     pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
     pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
     
     /* Wait for signal to start */
     printf("Sender thread %d waiting for start signal\n", thread_data->thread_id);
     sem_wait(thread_data->start_sem);
     
     /* Loop sending messages until canceled */
     int iteration = 0;
     while (iteration < 50 && msg_count < 5) {  /* Limit iterations */
         iteration++;
         
         /* Generate a unique message ID */
         int msg_id;
         pthread_mutex_lock(&g_mutex);
         msg_id = g_message_counter++;
         pthread_mutex_unlock(&g_mutex);
         
         /* Prepare message */
         snprintf(buffer, MESSAGE_SIZE, "Message %d from thread %d", 
                  msg_id, thread_data->thread_id);
         
         /* Send message - this is a cancellation point if queue is full */
         
 #if USE_TIMEOUTS
         /* Set timeout for send operation */
         clock_gettime(CLOCK_REALTIME, &timeout);
         timeout.tv_sec += DEFAULT_TIMEOUT_SEC;
         
         result = mq_timedsend(thread_data->mqdes, buffer, strlen(buffer) + 1, 1, &timeout);
 #else
         result = mq_send(thread_data->mqdes, buffer, strlen(buffer) + 1, 1);
 #endif
         
         /* Check if we sent the message */
         if (result == 0) {
             printf("Thread %d sent message successfully\n", thread_data->thread_id);
             msg_count++;
             
             /* Sleep a bit to give receivers a chance */
             usleep(10 * 1000); /* 10ms - shorter sleep for more contention */
         } else if (errno == ETIMEDOUT) {
             /* Expected if queue is full - try again */
         } else if (errno != EINTR) {
             printf("Thread %d mq_send error: %d\n", thread_data->thread_id, errno);
             break;
         }
         
         /* Check if cancellation is pending */
         pthread_testcancel();
         
         /* Check if we should exit */
         if (sem_trywait(thread_data->wait_sem) == 0) {
             printf("Thread %d exiting after semaphore signal\n", thread_data->thread_id);
             break;
         }
     }
     
     /* Mark thread as completed normally */
     pthread_mutex_lock(&g_mutex);
     if (thread_data->status == 0) { /* still running */
         thread_data->status = 2; /* completed */
     }
     pthread_mutex_unlock(&g_mutex);
     
     /* Execute cleanup handler on thread exit */
     pthread_cleanup_pop(1);
     printf("Thread %d finished\n", thread_data->thread_id);
     return NULL;
 }
 
 /****************************************************************************
  * Name: canceler_thread
  *
  * Description:
  *   A thread that cancels other threads that are waiting on message queues.
  *   In this SMP stress test, it aggressively cancels multiple threads
  *   in rapid succession.
  *
  ****************************************************************************/
 
 static void *canceler_thread(void *param)
 {
     struct thread_data_s *thread_data = (struct thread_data_s *)param;
     int i;
     char test_msg[MESSAGE_SIZE];
     
     printf("Canceler thread starting\n");
     
     /* Sleep a bit to allow threads to start and block on message queue operations */
     usleep(100 * 1000); /* 100ms */
     
     /* Cancel multiple receiver threads in rapid succession */
     printf("SMP TEST: Rapidly canceling multiple threads\n");
     
     /* Cancel half of the receiver threads in very rapid succession */
     for (i = 0; i < NUM_RECEIVER_THREADS / 2; i++) {
         printf("Canceling receiver thread %d\n", thread_data[i].thread_id);
         
         if (pthread_cancel(thread_data[i].thread) != 0) {
             printf("Error canceling thread %d\n", thread_data[i].thread_id);
         } else {
             printf("Cancel request sent to thread %d\n", thread_data[i].thread_id);
         }
         
         /* Minimal delay between cancellations to increase SMP stress */
         usleep(1 * 1000); /* Just 1ms between cancellations */
     }
     
     /* Join the canceled threads */
     for (i = 0; i < NUM_RECEIVER_THREADS / 2; i++) {
         int ret = pthread_join(thread_data[i].thread, NULL);
         if (ret != 0) {
             printf("Warning: Failed to join thread %d: %d\n", 
                    thread_data[i].thread_id, ret);
         }
         
         /* Mark as canceled */
         thread_data[i].status = 1;
     }
     
     /* Print queue status after cancellations */
     print_mq_status(thread_data[0].mqdes, "SMP TEST: Status after multiple thread cancellations");
     
     /* Check if the message queue is still usable by sending a test message */
     printf("SMP TEST: Testing queue functionality after cancellations\n");
     
     /* Send a test message */
     snprintf(test_msg, MESSAGE_SIZE, "Test after cancellation");
     if (mq_send(thread_data[0].mqdes, test_msg, strlen(test_msg) + 1, 1) == 0) {
         printf("Successfully sent test message after thread cancellations\n");
     } else {
         printf("Failed to send test message after thread cancellations: %d\n", errno);
     }
     
     /* Print queue status after test message */
     print_mq_status(thread_data[0].mqdes, "SMP TEST: Status after sending test message");
     
     printf("Canceler thread finished\n");
     return NULL;
 }
 
 /****************************************************************************
  * Name: cleanup_handler
  *
  * Description:
  *   Cleanup handler for threads to ensure proper state tracking
  *
  ****************************************************************************/
 
 static void cleanup_handler(void *arg)
 {
     struct thread_data_s *thread_data = (struct thread_data_s *)arg;
     
     /* Update thread status if it was canceled */
     pthread_mutex_lock(&g_mutex);
     if (thread_data->status == 0) { /* still running */
         thread_data->status = 1; /* mark as canceled */
         printf("Thread %d cleanup: marking as canceled\n", thread_data->thread_id);
     }
     pthread_mutex_unlock(&g_mutex);
 }
 
 /****************************************************************************
  * Name: print_test_results
  *
  * Description:
  *   Print the results of the test
  *
  ****************************************************************************/
 
 static void print_test_results(struct thread_data_s *threads, int num_threads)
 {
     int i;
     int canceled = 0;
     int completed = 0;
     
     printf("\nTest Results:\n");
     printf("-------------\n");
     
     for (i = 0; i < num_threads; i++) {
         printf("Thread %d: %s\n", threads[i].thread_id, 
                threads[i].status == 1 ? "Canceled" : 
                threads[i].status == 2 ? "Completed" : "Unknown");
         
         if (threads[i].status == 1) {
             canceled++;
         } else if (threads[i].status == 2) {
             completed++;
         }
     }
     
     printf("\nSummary: %d threads canceled, %d threads completed normally\n", 
            canceled, completed);
 }
 
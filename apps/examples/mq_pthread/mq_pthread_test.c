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
 
 /* Use timeouts to prevent blocking forever */
 #define USE_TIMEOUTS 1
 
 /* Safety timeout values */
 #define DEFAULT_TIMEOUT_SEC     2   /* General timeout for operations in seconds */
 
 /* Test parameters */
 #define MQ_NAME                  "/mq_pthread_cancel_test"
 #define EMPTY_QUEUE_NAME         "/mq_empty_queue"
 #define FULL_QUEUE_NAME          "/mq_full_queue"
 
 /* Reduce the number of threads and iterations for better stability */
 #define NUM_RECEIVER_THREADS     5
 #define NUM_SENDER_THREADS       3
 #define NUM_TEST_ITERATIONS      5
 #define TEST_QUEUE_MAX_MSGS      4        /* Small queue size to make it fill up quickly */
 #define MESSAGE_SIZE             32
 #define SEND_ATTEMPTS_PER_THREAD 5
 #define STRESS_SENDERS_FIRST     1
 
 /* Testing status reporting */
 #define STATUS_INTERVAL_MS       500      /* Status report interval in ms */
 
 /****************************************************************************
  * Private Types
  ****************************************************************************/
 
 enum test_phase_e {
     PHASE_NONE = 0,
     PHASE_BASIC_TEST,
     PHASE_EMPTY_QUEUE_TEST,
     PHASE_FULL_QUEUE_TEST,
     PHASE_COMPLETE
 };
 
 struct thread_data_s {
     int thread_id;
     pthread_t thread;
     int is_receiver;
     mqd_t mqdes;
     int cancel_after_ms;
     sem_t *wait_sem;       /* Semaphore for synchronization */
     sem_t *start_sem;      /* Semaphore for synchronized start */
     volatile int status;   /* 0 = running, 1 = canceled, 2 = completed */
 };
 
 /****************************************************************************
  * Private Function Prototypes
  ****************************************************************************/
 
 static void *receiver_thread(void *param);
 static void *sender_thread(void *param);
 static void *sender_stress_thread(void *param);
 static void *canceler_thread(void *param);
 static void *status_reporter_thread(void *param);
 static void cleanup_handler(void *arg);
 static void print_test_results(struct thread_data_s *threads, int num_threads);
 static void print_mq_status(mqd_t mqdes, const char *label);
 static int run_basic_cancellation_test(void);
 static int run_empty_queue_stress_test(void);
 static int run_full_queue_stress_test(void);
 
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
         "NONE", "BASIC_TEST", "EMPTY_QUEUE_TEST", "FULL_QUEUE_TEST", "COMPLETE"
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
 
     printf("Message Queue pthread_cancel Test\n");
     printf("==================================\n");
 
     pthread_mutexattr_init(&attr);
     pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
     pthread_mutex_init(&g_mutex, &attr);
     pthread_mutexattr_destroy(&attr);
     
     /* Start the status reporter thread */
     g_status_report_active = 1;
     if (pthread_create(&status_thread, NULL, status_reporter_thread, NULL) != 0) {
         printf("WARNING: Failed to create status reporter thread\n");
     }
 
     /* Run basic test */
     printf("\nRunning basic cancellation test...\n");
     g_current_phase = PHASE_BASIC_TEST;
     ret = run_basic_cancellation_test();
     if (ret != 0) {
         printf("Basic cancellation test failed with error %d\n", ret);
         g_status_report_active = 0;
         pthread_join(status_thread, NULL);
         pthread_mutex_destroy(&g_mutex);
         return -1;
     }
     printf("Basic cancellation test completed successfully\n");
 
     /* Run empty queue stress test */
     printf("\nRunning empty queue stress test...\n");
     g_current_phase = PHASE_EMPTY_QUEUE_TEST;
     ret = run_empty_queue_stress_test();
     if (ret != 0) {
         printf("Empty queue stress test failed with error %d\n", ret);
         g_status_report_active = 0;
         pthread_join(status_thread, NULL);
         pthread_mutex_destroy(&g_mutex);
         return -1;
     }
     printf("Empty queue stress test completed successfully\n");
 
     /* Run full queue stress test */
     printf("\nRunning full queue stress test...\n");
     g_current_phase = PHASE_FULL_QUEUE_TEST;
     ret = run_full_queue_stress_test();
     if (ret != 0) {
         printf("Full queue stress test failed with error %d\n", ret);
         g_status_report_active = 0;
         pthread_join(status_thread, NULL);
         pthread_mutex_destroy(&g_mutex);
         return -1;
     }
     printf("Full queue stress test completed successfully\n");
 
     /* Stop the status reporter thread */
     g_current_phase = PHASE_COMPLETE;
     g_status_report_active = 0;
     pthread_join(status_thread, NULL);
     
     pthread_mutex_destroy(&g_mutex);
     
     printf("\nAll tests completed successfully!\n");
     return 0;
 }
 
 /****************************************************************************
  * Name: run_basic_cancellation_test
  *
  * Description:
  *   Run a basic test with a few threads being canceled while waiting for
  *   message queue operations.
  *
  * Return Value:
  *   0 (OK) or -1 (ERROR) if test failed
  *
  ****************************************************************************/
 
 static int run_basic_cancellation_test(void)
 {
     int ret;
     int i;
     pthread_t canceler_thread_id;
     struct mq_attr attr;
     mqd_t mqdes;
     struct thread_data_s thread_data[5]; /* 3 receivers, 2 senders */
     sem_t wait_sem;
     sem_t start_sem;
     
     printf("BASIC TEST: Initializing test resources\n");
     
     /* Initialize semaphores */
     sem_init(&wait_sem, 0, 0);
     sem_init(&start_sem, 0, 0);
     
     /* Create message queue */
     attr.mq_maxmsg = TEST_QUEUE_MAX_MSGS;
     attr.mq_msgsize = MESSAGE_SIZE;
     attr.mq_flags = 0;
     
     /* Unlink any existing queues with these names */
     mq_unlink(MQ_NAME);
     
     /* Create the message queue */
     printf("BASIC TEST: Creating message queue %s\n", MQ_NAME);
     mqdes = mq_open(MQ_NAME, O_CREAT | O_RDWR, 0666, &attr);
     if (mqdes == (mqd_t)-1) {
         printf("Error opening message queue: %d\n", errno);
         sem_destroy(&wait_sem);
         sem_destroy(&start_sem);
         return -1;
     }
     
     /* Store the active message queue descriptor for status reports */
     g_active_mqdes = mqdes;
     
     g_test_counter++;
     
     /* Set up the thread data and create threads */
     printf("BASIC TEST: Creating worker threads\n");
     for (i = 0; i < 5; i++) {
         thread_data[i].thread_id = i;
         thread_data[i].mqdes = mqdes;
         thread_data[i].cancel_after_ms = 100 + (rand() % 200); /* 100-300ms */
         thread_data[i].wait_sem = &wait_sem;
         thread_data[i].start_sem = &start_sem;
         thread_data[i].status = 0; /* running */
         
         /* First 3 are receivers, last 2 are senders */
         thread_data[i].is_receiver = (i < 3);
         
         if (thread_data[i].is_receiver) {
             ret = pthread_create(&thread_data[i].thread, NULL, receiver_thread, &thread_data[i]);
         } else {
             ret = pthread_create(&thread_data[i].thread, NULL, sender_thread, &thread_data[i]);
         }
         
         if (ret != 0) {
             printf("Error creating thread %d: %d\n", i, ret);
             mq_close(mqdes);
             mq_unlink(MQ_NAME);
             sem_destroy(&wait_sem);
             sem_destroy(&start_sem);
             return -1;
         }
     }
     
     /* Make sure all threads have started and are waiting */
     usleep(200 * 1000); /* 200ms */
     
     /* Release threads to start working */
     printf("BASIC TEST: Starting worker threads\n");
     for (i = 0; i < 5; i++) {
         sem_post(&start_sem);
     }
     
     /* Print initial queue status */
     print_mq_status(mqdes, "BASIC TEST: Initial queue status");
     
     /* Create canceler thread */
     printf("BASIC TEST: Creating canceler thread\n");
     ret = pthread_create(&canceler_thread_id, NULL, canceler_thread, thread_data);
     if (ret != 0) {
         printf("Error creating canceler thread: %d\n", ret);
         mq_close(mqdes);
         mq_unlink(MQ_NAME);
         sem_destroy(&wait_sem);
         sem_destroy(&start_sem);
         return -1;
     }
     
     /* Wait for canceler thread to complete */
     printf("BASIC TEST: Waiting for canceler thread to complete\n");
     ret = pthread_join(canceler_thread_id, NULL);
     if (ret != 0) {
         printf("BASIC TEST: Error joining canceler thread: %d\n", ret);
     } else {
         printf("BASIC TEST: Canceler thread completed successfully\n");
     }
     
     /* Signal any remaining threads to exit */
     printf("BASIC TEST: Signaling any remaining threads to exit\n");
     for (i = 0; i < 5; i++) {
         sem_post(&wait_sem);
     }
     
     /* Wait for any remaining threads to complete */
     printf("BASIC TEST: Waiting for any remaining threads to complete\n");
     for (i = 0; i < 5; i++) {
         if (thread_data[i].status == 0) { /* still running */
             printf("BASIC TEST: Waiting for thread %d to exit\n", i);
             pthread_join(thread_data[i].thread, NULL);
             printf("BASIC TEST: Thread %d has exited\n", i);
         }
     }
     
     /* Print test results */
     print_test_results(thread_data, 5);
     
     /* Print final queue status */
     print_mq_status(mqdes, "BASIC TEST: Final queue status");
     
     /* Clean up resources */
     printf("BASIC TEST: Cleaning up resources\n");
     g_active_mqdes = 0;
     mq_close(mqdes);
     mq_unlink(MQ_NAME);
     sem_destroy(&wait_sem);
     sem_destroy(&start_sem);
     
     printf("BASIC TEST: Successfully completed\n");
     return 0;
 }
 
 /****************************************************************************
  * Name: run_empty_queue_stress_test
  *
  * Description:
  *   Run a stress test focusing on canceling many threads that are waiting
  *   to receive from an empty queue.
  *
  * Return Value:
  *   0 (OK) or -1 (ERROR) if test failed
  *
  ****************************************************************************/
 
 static int run_empty_queue_stress_test(void)
 {
     int ret;
     int i, j;
     struct mq_attr attr;
     mqd_t mqdes;
     struct thread_data_s thread_data[NUM_RECEIVER_THREADS];
     sem_t wait_sem;
     sem_t start_sem;
     
     printf("EMPTY QUEUE TEST: Initializing test resources\n");
     
     /* Initialize semaphores */
     sem_init(&wait_sem, 0, 0);
     sem_init(&start_sem, 0, 0);
     
     /* Create message queue */
     attr.mq_maxmsg = TEST_QUEUE_MAX_MSGS;
     attr.mq_msgsize = MESSAGE_SIZE;
     attr.mq_flags = 0;
     
     /* Unlink any existing queues with this name */
     mq_unlink(EMPTY_QUEUE_NAME);
     
     /* Create an empty message queue */
     printf("EMPTY QUEUE TEST: Creating message queue %s\n", EMPTY_QUEUE_NAME);
     mqdes = mq_open(EMPTY_QUEUE_NAME, O_CREAT | O_RDWR, 0666, &attr);
     if (mqdes == (mqd_t)-1) {
         printf("Error opening empty message queue: %d\n", errno);
         sem_destroy(&wait_sem);
         sem_destroy(&start_sem);
         return -1;
     }
     
     /* Store the active message queue descriptor for status reports */
     g_active_mqdes = mqdes;
     
     /* Run multiple iterations of the test */
     for (j = 0; j < NUM_TEST_ITERATIONS; j++) {
         printf("\nEMPTY QUEUE TEST: Iteration %d/%d\n", j+1, NUM_TEST_ITERATIONS);
         
         /* Create multiple receiver threads to wait on the empty queue */
         printf("EMPTY QUEUE TEST: Creating receiver threads\n");
         for (i = 0; i < NUM_RECEIVER_THREADS; i++) {
             thread_data[i].thread_id = i;
             thread_data[i].mqdes = mqdes;
             thread_data[i].is_receiver = 1;
             thread_data[i].wait_sem = &wait_sem;
             thread_data[i].start_sem = &start_sem;
             thread_data[i].status = 0; /* running */
             
             ret = pthread_create(&thread_data[i].thread, NULL, receiver_thread, &thread_data[i]);
             if (ret != 0) {
                 printf("Error creating receiver thread %d: %d\n", i, ret);
                 mq_close(mqdes);
                 mq_unlink(EMPTY_QUEUE_NAME);
                 sem_destroy(&wait_sem);
                 sem_destroy(&start_sem);
                 return -1;
             }
         }
         
         /* Allow threads to start and get blocked on receive */
         usleep(100 * 1000); /* 100ms */
         
         /* Release threads to start waiting on message queue */
         printf("EMPTY QUEUE TEST: Starting receiver threads\n");
         for (i = 0; i < NUM_RECEIVER_THREADS; i++) {
             sem_post(&start_sem);
         }
         
         /* Give threads time to get blocked on mq_receive */
         usleep(200 * 1000); /* 200ms */
         
         /* Print queue status before thread cancellation */
         print_mq_status(mqdes, "EMPTY QUEUE TEST: Status before thread cancellation");
         
         /* Cancel all threads */
         printf("EMPTY QUEUE TEST: Canceling all receiver threads\n");
         for (i = 0; i < NUM_RECEIVER_THREADS; i++) {
             printf("Canceling thread %d\n", i);
             ret = pthread_cancel(thread_data[i].thread);
             if (ret != 0) {
                 printf("Error canceling thread %d: %d\n", i, ret);
             }
         }
         
         /* Join all threads */
         printf("EMPTY QUEUE TEST: Joining all receiver threads\n");
         for (i = 0; i < NUM_RECEIVER_THREADS; i++) {
             ret = pthread_join(thread_data[i].thread, NULL);
             if (ret != 0) {
                 printf("Error joining thread %d: %d\n", i, ret);
             }
             thread_data[i].status = 1; /* mark as canceled */
         }
         
         /* Print queue status after thread cancellation */
         print_mq_status(mqdes, "EMPTY QUEUE TEST: Status after thread cancellation");
         
         /* Now try to send a message to make sure the queue is still usable */
         printf("EMPTY QUEUE TEST: Sending test message\n");
         char test_msg[MESSAGE_SIZE];
         snprintf(test_msg, MESSAGE_SIZE, "Test message after cancellation %d", j);
         
         ret = mq_send(mqdes, test_msg, strlen(test_msg) + 1, 1);
         if (ret != 0) {
             printf("Failed to send test message after cancellation: %d\n", errno);
             mq_close(mqdes);
             mq_unlink(EMPTY_QUEUE_NAME);
             sem_destroy(&wait_sem);
             sem_destroy(&start_sem);
             return -1;
         }
         
         /* Print queue status after sending test message */
         print_mq_status(mqdes, "EMPTY QUEUE TEST: Status after sending test message");
         
         /* Drain the queue */
         printf("EMPTY QUEUE TEST: Draining the queue\n");
         char buffer[MESSAGE_SIZE];
         unsigned int prio;
         struct timespec timeout;
         
         clock_gettime(CLOCK_REALTIME, &timeout);
         timeout.tv_sec += DEFAULT_TIMEOUT_SEC;
         
         if (mq_timedreceive(mqdes, buffer, MESSAGE_SIZE, &prio, &timeout) < 0) {
             printf("Failed to receive message after cancellation: %d\n", errno);
         } else {
             printf("Successfully received: %s\n", buffer);
         }
         
         /* Print queue status after draining */
         print_mq_status(mqdes, "EMPTY QUEUE TEST: Status after draining queue");
         
         /* If we got here without an assertion failure, the test passed this iteration */
         printf("EMPTY QUEUE TEST: Iteration %d completed\n", j+1);
     }
     
     /* Clean up resources */
     printf("EMPTY QUEUE TEST: Cleaning up resources\n");
     g_active_mqdes = 0;
     mq_close(mqdes);
     mq_unlink(EMPTY_QUEUE_NAME);
     sem_destroy(&wait_sem);
     sem_destroy(&start_sem);
     
     printf("EMPTY QUEUE TEST: Successfully completed\n");
     return 0;
 }
 
 /****************************************************************************
  * Name: run_full_queue_stress_test
  *
  * Description:
  *   Run a stress test focusing on canceling many threads that are waiting
  *   to send to a full queue.
  *
  * Return Value:
  *   0 (OK) or -1 (ERROR) if test failed
  *
  ****************************************************************************/
 
 static int run_full_queue_stress_test(void)
 {
     int ret;
     int i, j;
     struct mq_attr attr;
     mqd_t mqdes;
     struct thread_data_s thread_data[NUM_SENDER_THREADS];
     sem_t wait_sem;
     sem_t start_sem;
     char buffer[MESSAGE_SIZE];
     
     printf("FULL QUEUE TEST: Initializing test resources\n");
     
     /* Initialize semaphores */
     sem_init(&wait_sem, 0, 0);
     sem_init(&start_sem, 0, 0);
     
     /* Create message queue with small capacity */
     attr.mq_maxmsg = TEST_QUEUE_MAX_MSGS;  /* Small queue */
     attr.mq_msgsize = MESSAGE_SIZE;
     attr.mq_flags = 0;
     
     /* Unlink any existing queues with this name */
     mq_unlink(FULL_QUEUE_NAME);
     
     /* Create the message queue */
     printf("FULL QUEUE TEST: Creating message queue %s\n", FULL_QUEUE_NAME);
     mqdes = mq_open(FULL_QUEUE_NAME, O_CREAT | O_RDWR, 0666, &attr);
     if (mqdes == (mqd_t)-1) {
         printf("Error opening full message queue: %d\n", errno);
         sem_destroy(&wait_sem);
         sem_destroy(&start_sem);
         return -1;
     }
     
     /* Store the active message queue descriptor for status reports */
     g_active_mqdes = mqdes;
     
     /* Run multiple iterations of the test */
     for (j = 0; j < NUM_TEST_ITERATIONS; j++) {
         printf("\nFULL QUEUE TEST: Iteration %d/%d\n", j+1, NUM_TEST_ITERATIONS);
         
         /* Fill the queue to capacity */
         printf("FULL QUEUE TEST: Filling queue to capacity\n");
         for (i = 0; i < TEST_QUEUE_MAX_MSGS; i++) {
             snprintf(buffer, MESSAGE_SIZE, "Fill message %d", i);
             ret = mq_send(mqdes, buffer, strlen(buffer) + 1, 1);
             if (ret != 0) {
                 printf("Failed to fill queue: %d\n", errno);
                 mq_close(mqdes);
                 mq_unlink(FULL_QUEUE_NAME);
                 sem_destroy(&wait_sem);
                 sem_destroy(&start_sem);
                 return -1;
             }
         }
         
         /* Print queue status after filling */
         print_mq_status(mqdes, "FULL QUEUE TEST: Status after filling queue");
         
         /* Create multiple sender threads to wait on the full queue */
         printf("FULL QUEUE TEST: Creating sender threads\n");
         for (i = 0; i < NUM_SENDER_THREADS; i++) {
             thread_data[i].thread_id = i;
             thread_data[i].mqdes = mqdes;
             thread_data[i].is_receiver = 0;
             thread_data[i].wait_sem = &wait_sem;
             thread_data[i].start_sem = &start_sem;
             thread_data[i].status = 0; /* running */
             
             ret = pthread_create(&thread_data[i].thread, NULL, sender_stress_thread, &thread_data[i]);
             if (ret != 0) {
                 printf("Error creating sender thread %d: %d\n", i, ret);
                 mq_close(mqdes);
                 mq_unlink(FULL_QUEUE_NAME);
                 sem_destroy(&wait_sem);
                 sem_destroy(&start_sem);
                 return -1;
             }
         }
         
         /* Allow threads to start and get blocked on send */
         usleep(100 * 1000); /* 100ms */
         
         /* Release threads to start waiting on message queue */
         printf("FULL QUEUE TEST: Starting sender threads\n");
         for (i = 0; i < NUM_SENDER_THREADS; i++) {
             sem_post(&start_sem);
         }
         
         /* Give threads time to get blocked on mq_send */
         usleep(200 * 1000); /* 200ms */
         
         /* Print queue status before thread cancellation */
         print_mq_status(mqdes, "FULL QUEUE TEST: Status before thread cancellation");
         
         /* Cancel all threads */
         printf("FULL QUEUE TEST: Canceling all sender threads\n");
         for (i = 0; i < NUM_SENDER_THREADS; i++) {
             printf("Canceling sender thread %d\n", i);
             ret = pthread_cancel(thread_data[i].thread);
             if (ret != 0) {
                 printf("Error canceling thread %d: %d\n", i, ret);
             }
         }
         
         /* Join all threads */
         printf("FULL QUEUE TEST: Joining all sender threads\n");
         for (i = 0; i < NUM_SENDER_THREADS; i++) {
             ret = pthread_join(thread_data[i].thread, NULL);
             if (ret != 0) {
                 printf("Error joining thread %d: %d\n", i, ret);
             }
             thread_data[i].status = 1; /* mark as canceled */
         }
         
         /* Print queue status after thread cancellation */
         print_mq_status(mqdes, "FULL QUEUE TEST: Status after thread cancellation");
         
         /* Now try to receive a message to make sure the queue is still usable */
         printf("FULL QUEUE TEST: Receiving test message\n");
         unsigned int prio;
         struct timespec timeout;
         
         clock_gettime(CLOCK_REALTIME, &timeout);
         timeout.tv_sec += DEFAULT_TIMEOUT_SEC;
         
         ret = mq_timedreceive(mqdes, buffer, MESSAGE_SIZE, &prio, &timeout);
         if (ret < 0) {
             printf("Failed to receive message after cancellation: %d\n", errno);
             mq_close(mqdes);
             mq_unlink(FULL_QUEUE_NAME);
             sem_destroy(&wait_sem);
             sem_destroy(&start_sem);
             return -1;
         } else {
             printf("Successfully received: %s\n", buffer);
         }
         
         /* Print queue status after receiving */
         print_mq_status(mqdes, "FULL QUEUE TEST: Status after receiving message");
         
         /* Drain the rest of the queue */
         printf("FULL QUEUE TEST: Draining the remaining messages\n");
         while (1) {
             clock_gettime(CLOCK_REALTIME, &timeout);
             timeout.tv_sec += 1;
             
             ret = mq_timedreceive(mqdes, buffer, MESSAGE_SIZE, &prio, &timeout);
             if (ret < 0) {
                 /* Break when the queue is empty or on error */
                 printf("Queue drain complete or error: %d\n", errno);
                 break;
             } else {
                 printf("Drained message: %s\n", buffer);
             }
         }
         
         /* Print queue status after draining */
         print_mq_status(mqdes, "FULL QUEUE TEST: Status after draining queue");
         
         /* If we got here without an assertion failure, the test passed this iteration */
         printf("FULL QUEUE TEST: Iteration %d completed\n", j+1);
     }
     
     /* Clean up resources */
     printf("FULL QUEUE TEST: Cleaning up resources\n");
     g_active_mqdes = 0;
     mq_close(mqdes);
     mq_unlink(FULL_QUEUE_NAME);
     sem_destroy(&wait_sem);
     sem_destroy(&start_sem);
     
     printf("FULL QUEUE TEST: Successfully completed\n");
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
     
     printf("Receiver thread %d starting\n", thread_data->thread_id);
     
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
     while (iteration < 10) {  /* Limit iterations to prevent infinite loops */
         iteration++;
         
         printf("Thread %d waiting iteration %d\n", thread_data->thread_id, iteration);
         
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
             printf("Thread %d received message: %.*s\n", thread_data->thread_id, result, buffer);
             break;
         } else if (errno == ETIMEDOUT) {
             printf("Thread %d receive timed out\n", thread_data->thread_id);
             /* Try again */
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
     
     printf("Thread %d exiting loop\n", thread_data->thread_id);
     
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
     
     printf("Sender thread %d starting\n", thread_data->thread_id);
     
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
     while (iteration < 10 && msg_count < 5) {  /* Limit iterations */
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
         printf("Thread %d sending message: %s\n", thread_data->thread_id, buffer);
         
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
             usleep(50 * 1000); /* 50ms */
         } else if (errno == ETIMEDOUT) {
             printf("Thread %d send timed out\n", thread_data->thread_id);
             /* Try again */
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
     
     printf("Thread %d exiting loop\n", thread_data->thread_id);
     
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
  * Name: sender_stress_thread
  *
  * Description:
  *   A thread that repeatedly tries to send messages to a full queue.
  *   This is used for stress testing thread cancellation on full queues.
  *
  ****************************************************************************/
 
 static void *sender_stress_thread(void *param)
 {
     struct thread_data_s *thread_data = (struct thread_data_s *)param;
     char buffer[MESSAGE_SIZE];
     int attempt;
     struct timespec timeout;
     
     printf("Sender stress thread %d starting\n", thread_data->thread_id);
     
     /* Install cleanup handler */
     pthread_cleanup_push(cleanup_handler, thread_data);
     
     /* Make thread cancelable */
     pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
     pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
     
     /* Wait for signal to start */
     printf("Sender stress thread %d waiting for start signal\n", thread_data->thread_id);
     sem_wait(thread_data->start_sem);
     printf("Sender stress thread %d starting to send\n", thread_data->thread_id);
     
     /* Try sending messages multiple times (will block because queue is full) */
     for (attempt = 0; attempt < SEND_ATTEMPTS_PER_THREAD; attempt++) {
         /* Check for cancellation */
         pthread_testcancel();
         
         /* Generate a unique message ID */
         int msg_id;
         pthread_mutex_lock(&g_mutex);
         msg_id = g_message_counter++;
         pthread_mutex_unlock(&g_mutex);
         
         /* Prepare message */
         snprintf(buffer, MESSAGE_SIZE, "Message %d from thread %d", 
                  msg_id, thread_data->thread_id);
         
         /* Send message - this will block if queue is full */
         printf("Thread %d trying to send to full queue: %s\n", 
                thread_data->thread_id, buffer);
         
 #if USE_TIMEOUTS
         /* Set timeout for send operation */
         clock_gettime(CLOCK_REALTIME, &timeout);
         timeout.tv_sec += DEFAULT_TIMEOUT_SEC;
         
         int ret = mq_timedsend(thread_data->mqdes, buffer, strlen(buffer) + 1, 1, &timeout);
         if (ret == 0) {
             printf("Thread %d sent to full queue successfully (unexpected)\n", thread_data->thread_id);
         } else if (errno == ETIMEDOUT) {
             printf("Thread %d send timed out (expected for full queue)\n", thread_data->thread_id);
         } else {
             printf("Thread %d mq_timedsend error: %d\n", thread_data->thread_id, errno);
         }
 #else
         /* This may block forever if the queue is full */
         mq_send(thread_data->mqdes, buffer, strlen(buffer) + 1, 1);
 #endif
         
         /* Check for cancellation again */
         pthread_testcancel();
     }
     
     /* We should rarely reach here because we'll be canceled or blocked */
     printf("Thread %d completed all send attempts - unexpected!\n", thread_data->thread_id);
     
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
  *
  ****************************************************************************/
 
 static void *canceler_thread(void *param)
 {
     struct thread_data_s *thread_data = (struct thread_data_s *)param;
     int i;
     
     printf("Canceler thread starting\n");
     
     /* Sleep to allow threads to start and block on message queue operations */
     usleep(500 * 1000); /* 500ms */
     
     /* Cancel 3 of the 5 threads at random times */
     for (i = 0; i < 3; i++) {
         int thread_to_cancel = rand() % 5;
         
         /* Sleep a bit before canceling */
         usleep(100 * 1000); /* 100ms */
         
         printf("Canceler thread: About to cancel thread %d [phase %d]\n", 
                thread_data[thread_to_cancel].thread_id, g_current_phase);
         
         if (pthread_cancel(thread_data[thread_to_cancel].thread) != 0) {
             printf("Error canceling thread %d\n", thread_data[thread_to_cancel].thread_id);
         } else {
             printf("Successfully sent cancel request to thread %d\n", 
                    thread_data[thread_to_cancel].thread_id);
         }
         
         /* Wait for the thread to terminate */
         printf("Waiting for thread %d to terminate\n", thread_data[thread_to_cancel].thread_id);
         int ret = pthread_join(thread_data[thread_to_cancel].thread, NULL);
         if (ret != 0) {
             printf("Warning: Failed to join thread %d: %d\n", 
                    thread_data[thread_to_cancel].thread_id, ret);
         } else {
             printf("Successfully joined thread %d\n", thread_data[thread_to_cancel].thread_id);
         }
         
         /* Mark the thread as canceled */
         thread_data[thread_to_cancel].status = 1;
         
         /* Print message queue status after cancellation */
         print_mq_status(thread_data[0].mqdes, "Status after thread cancellation");
         
         /* Check if the message queue is still usable */
         char test_msg[MESSAGE_SIZE];
         snprintf(test_msg, MESSAGE_SIZE, "Test after cancel %d", i);
         
         printf("Canceler thread: Sending test message after cancellation\n");
         if (mq_send(thread_data[0].mqdes, test_msg, strlen(test_msg) + 1, 1) == 0) {
             printf("Successfully sent test message after canceling thread %d\n", 
                    thread_data[thread_to_cancel].thread_id);
         } else {
             printf("Failed to send test message after canceling thread %d: %d (errno=%d)\n", 
                    thread_data[thread_to_cancel].thread_id, ret, errno);
         }
         
         /* Print message queue status after sending test message */
         print_mq_status(thread_data[0].mqdes, "Status after sending test message");
     }
     
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
 
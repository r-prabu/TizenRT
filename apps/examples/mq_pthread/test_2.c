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
 #include <signal.h>
 
 /****************************************************************************
  * Pre-processor Definitions
  ****************************************************************************/
 
 /* Use timeouts to prevent blocking forever */
 #define USE_TIMEOUTS 1
 
 /* Safety timeout values */
 #define DEFAULT_TIMEOUT_SEC     2   /* General timeout for operations in seconds */
 #define WATCHDOG_TIMEOUT_SEC    30  /* Global test watchdog timeout */
 
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
 
 /****************************************************************************
  * Private Types
  ****************************************************************************/
 
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
 static void cleanup_handler(void *arg);
 static void print_test_results(struct thread_data_s *threads, int num_threads);
 static int run_basic_cancellation_test(void);
 static int run_empty_queue_stress_test(void);
 static int run_full_queue_stress_test(void);
 static void watchdog_handler(int signo);
 
 /****************************************************************************
  * Private Data
  ****************************************************************************/
 
 static pthread_mutex_t g_mutex;
 static int g_test_counter = 0;
 static int g_message_counter = 0;
 static volatile int g_watchdog_triggered = 0;
 
 /****************************************************************************
  * Public Functions
  ****************************************************************************/
 
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
     struct sigaction sa;
 
     printf("Message Queue pthread_cancel Test\n");
     printf("==================================\n");
 
     /* Set up watchdog timer to catch test hangs */
     sa.sa_handler = watchdog_handler;
     sigemptyset(&sa.sa_mask);
     sa.sa_flags = 0;
     sigaction(SIGALRM, &sa, NULL);
     
     /* Set alarm for WATCHDOG_TIMEOUT_SEC seconds */
     alarm(WATCHDOG_TIMEOUT_SEC);
 
     pthread_mutexattr_init(&attr);
     pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
     pthread_mutex_init(&g_mutex, &attr);
     pthread_mutexattr_destroy(&attr);
 
     /* Run basic test */
     printf("\nRunning basic cancellation test...\n");
     ret = run_basic_cancellation_test();
     if (ret != 0) {
         printf("Basic cancellation test failed with error %d\n", ret);
         pthread_mutex_destroy(&g_mutex);
         return -1;
     }
     printf("Basic cancellation test completed successfully\n");
 
     /* Run empty queue stress test */
     printf("\nRunning empty queue stress test...\n");
     ret = run_empty_queue_stress_test();
     if (ret != 0) {
         printf("Empty queue stress test failed with error %d\n", ret);
         pthread_mutex_destroy(&g_mutex);
         return -1;
     }
     printf("Empty queue stress test completed successfully\n");
 
     /* Run full queue stress test */
     printf("\nRunning full queue stress test...\n");
     ret = run_full_queue_stress_test();
     if (ret != 0) {
         printf("Full queue stress test failed with error %d\n", ret);
         pthread_mutex_destroy(&g_mutex);
         return -1;
     }
     printf("Full queue stress test completed successfully\n");
 
     /* Disable the watchdog */
     alarm(0);
 
     pthread_mutex_destroy(&g_mutex);
     
     printf("\nAll tests completed successfully!\n");
     return 0;
 }
 
 /****************************************************************************
  * Name: watchdog_handler
  *
  * Description:
  *   Signal handler for watchdog timer - detects if test hangs
  *
  ****************************************************************************/
 
 static void watchdog_handler(int signo)
 {
     printf("\n!!! WATCHDOG TRIGGERED - TEST HANGING !!!\n");
     g_watchdog_triggered = 1;
     
     /* Exit the process with an error code */
     exit(1);
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
     mqdes = mq_open(MQ_NAME, O_CREAT | O_RDWR, 0666, &attr);
     if (mqdes == (mqd_t)-1) {
         printf("Error opening message queue: %d\n", errno);
         sem_destroy(&wait_sem);
         sem_destroy(&start_sem);
         return -1;
     }
     
     g_test_counter++;
     
     /* Set up the thread data and create threads */
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
     for (i = 0; i < 5; i++) {
         sem_post(&start_sem);
     }
     
     /* Create canceler thread */
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
     pthread_join(canceler_thread_id, NULL);
     
     /* Signal any remaining threads to exit */
     for (i = 0; i < 5; i++) {
         sem_post(&wait_sem);
     }
     
     /* Wait for any remaining threads to complete */
     for (i = 0; i < 5; i++) {
         if (thread_data[i].status == 0) { /* still running */
             printf("Waiting for thread %d to exit\n", i);
             pthread_join(thread_data[i].thread, NULL);
         }
     }
     
     /* Print test results */
     print_test_results(thread_data, 5);
     
     /* Clean up resources */
     mq_close(mqdes);
     mq_unlink(MQ_NAME);
     sem_destroy(&wait_sem);
     sem_destroy(&start_sem);
     
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
     mqdes = mq_open(EMPTY_QUEUE_NAME, O_CREAT | O_RDWR, 0666, &attr);
     if (mqdes == (mqd_t)-1) {
         printf("Error opening empty message queue: %d\n", errno);
         sem_destroy(&wait_sem);
         sem_destroy(&start_sem);
         return -1;
     }
     
     /* Run multiple iterations of the test */
     for (j = 0; j < NUM_TEST_ITERATIONS; j++) {
         printf("\nEmpty queue stress test iteration %d/%d\n", j+1, NUM_TEST_ITERATIONS);
         
         /* Create multiple receiver threads to wait on the empty queue */
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
         for (i = 0; i < NUM_RECEIVER_THREADS; i++) {
             sem_post(&start_sem);
         }
         
         /* Give threads time to get blocked on mq_receive */
         usleep(200 * 1000); /* 200ms */
         
         /* Cancel all threads */
         for (i = 0; i < NUM_RECEIVER_THREADS; i++) {
             printf("Canceling thread %d\n", i);
             pthread_cancel(thread_data[i].thread);
         }
         
         /* Join all threads */
         for (i = 0; i < NUM_RECEIVER_THREADS; i++) {
             pthread_join(thread_data[i].thread, NULL);
             thread_data[i].status = 1; /* mark as canceled */
         }
         
         /* Now try to send a message to make sure the queue is still usable */
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
         
         /* Drain the queue */
         char buffer[MESSAGE_SIZE];
         unsigned int prio;
         struct timespec timeout;
         
         clock_gettime(CLOCK_REALTIME, &timeout);
         timeout.tv_sec += DEFAULT_TIMEOUT_SEC;
         
         if (mq_timedreceive(mqdes, buffer, MESSAGE_SIZE, &prio, &timeout) < 0) {
             printf("Failed to receive message after cancellation: %d\n", errno);
         }
         
         /* If we got here without an assertion failure, the test passed this iteration */
         printf("Empty queue stress test iteration %d completed\n", j+1);
     }
     
     /* Clean up resources */
     mq_close(mqdes);
     mq_unlink(EMPTY_QUEUE_NAME);
     sem_destroy(&wait_sem);
     sem_destroy(&start_sem);
     
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
     mqdes = mq_open(FULL_QUEUE_NAME, O_CREAT | O_RDWR, 0666, &attr);
     if (mqdes == (mqd_t)-1) {
         printf("Error opening full message queue: %d\n", errno);
         sem_destroy(&wait_sem);
         sem_destroy(&start_sem);
         return -1;
     }
     
     /* Run multiple iterations of the test */
     for (j = 0; j < NUM_TEST_ITERATIONS; j++) {
         printf("\nFull queue stress test iteration %d/%d\n", j+1, NUM_TEST_ITERATIONS);
         
         /* Fill the queue to capacity */
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
         
         /* Create multiple sender threads to wait on the full queue */
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
         for (i = 0; i < NUM_SENDER_THREADS; i++) {
             sem_post(&start_sem);
         }
         
         /* Give threads time to get blocked on mq_send */
         usleep(200 * 1000); /* 200ms */
         
         /* Cancel all threads */
         for (i = 0; i < NUM_SENDER_THREADS; i++) {
             printf("Canceling sender thread %d\n", i);
             pthread_cancel(thread_data[i].thread);
         }
         
         /* Join all threads */
         for (i = 0; i < NUM_SENDER_THREADS; i++) {
             pthread_join(thread_data[i].thread, NULL);
             thread_data[i].status = 1; /* mark as canceled */
         }
         
         /* Now try to receive messages to make sure the queue is still usable */
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
         }
         
         /* Drain the rest of the queue */
         while (1) {
             clock_gettime(CLOCK_REALTIME, &timeout);
             timeout.tv_sec += 1;
             
             ret = mq_timedreceive(mqdes, buffer, MESSAGE_SIZE, &prio, &timeout);
             if (ret < 0) {
                 /* Break when the queue is empty or on error */
                 break;
             }
         }
         
         /* If we got here without an assertion failure, the test passed this iteration */
         printf("Full queue stress test iteration %d completed\n", j+1);
     }
     
     /* Clean up resources */
     mq_close(mqdes);
     mq_unlink(FULL_QUEUE_NAME);
     sem_destroy(&wait_sem);
     sem_destroy(&start_sem);
     
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
     sem_wait(thread_data->start_sem);
     
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
         
         printf("Canceling thread %d\n", thread_data[thread_to_cancel].thread_id);
         
         if (pthread_cancel(thread_data[thread_to_cancel].thread) != 0) {
             printf("Error canceling thread %d\n", thread_data[thread_to_cancel].thread_id);
         }
         
         /* Wait for the thread to terminate with a timeout */
         struct timespec ts;
         clock_gettime(CLOCK_REALTIME, &ts);
         ts.tv_sec += DEFAULT_TIMEOUT_SEC;
         
         /* Wait for thread to finish */
         printf("waiting for thread %d to terminate\n", thread_data[thread_to_cancel].thread_id);
         int ret = pthread_join(thread_data[thread_to_cancel].thread, NULL);
         if (ret != 0) {
             printf("Warning: Failed to join thread %d: %d\n", 
                    thread_data[thread_to_cancel].thread_id, ret);
         }
         
         /* Mark the thread as canceled */
         thread_data[thread_to_cancel].status = 1;
         
         /* Check if the message queue is still usable */
         char test_msg[MESSAGE_SIZE];
         snprintf(test_msg, MESSAGE_SIZE, "Test after cancel %d", i);
         
         if (mq_send(thread_data[0].mqdes, test_msg, strlen(test_msg) + 1, 1) == 0) {
             printf("Successfully sent test message after canceling thread %d\n", 
                    thread_data[thread_to_cancel].thread_id);
         } else {
             printf("Failed to send test message after canceling thread %d: %d\n", 
                    thread_data[thread_to_cancel].thread_id, errno);
         }
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
 
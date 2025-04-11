/****************************************************************************
 *
 * Copyright 2025 Samsung Electronics All Rights Reserved.
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
 
 /****************************************************************************
  * Pre-processor Definitions
  ****************************************************************************/
 
 #define MQ_NAME                  "/mq_pthread_cancel_test"
 #define NUM_THREADS              5
 #define NUM_CANCELLATIONS        10
 #define NUM_MESSAGES             20
 #define MESSAGE_SIZE             32
 #define SLEEP_BETWEEN_TESTS_MS   500
 
 /****************************************************************************
  * Private Types
  ****************************************************************************/
 
 struct thread_data_s {
     int thread_id;
     pthread_t thread;
     mqd_t mqdes;
     int cancel_after_ms;
     int status;  /* 0 = running, 1 = canceled, 2 = completed */
 };
 
 /****************************************************************************
  * Private Function Prototypes
  ****************************************************************************/
 
 static void *mq_receiver_thread(void *param);
 static void *mq_sender_thread(void *param);
 static void *thread_canceler(void *param);
 static void cleanup_handler(void *arg);
 static void print_test_results(struct thread_data_s *threads, int num_threads);
 static int run_cancellation_test(void);
 
 /****************************************************************************
  * Private Data
  ****************************************************************************/
 
 static pthread_mutex_t g_mutex;
 static int g_test_counter = 0;
 
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
     int i;
     int ret = 0;
     pthread_mutexattr_t attr;
 
     printf("Message Queue pthread_cancel Test\n");
     printf("==================================\n");
 
     pthread_mutexattr_init(&attr);
     pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
     pthread_mutex_init(&g_mutex, &attr);
     pthread_mutexattr_destroy(&attr);
 
     /* Run the test multiple times to increase chances of catching the issue */
     for (i = 0; i < NUM_CANCELLATIONS; i++) {
         printf("\nTest Iteration %d of %d\n", i + 1, NUM_CANCELLATIONS);
         printf("----------------------------------\n");
         ret = run_cancellation_test();
         if (ret != 0) {
             printf("Test iteration %d failed with error %d\n", i + 1, ret);
             break;
         }
         
         /* Sleep between test iterations to allow for cleanup */
         usleep(SLEEP_BETWEEN_TESTS_MS * 1000);
     }
 
     pthread_mutex_destroy(&g_mutex);
     
     if (ret == 0) {
         printf("\nAll tests completed successfully!\n");
     }
 
     return ret;
 }
 
 /****************************************************************************
  * Name: run_cancellation_test
  *
  * Description:
  *   Run a single iteration of the pthread cancellation test.
  *
  * Return Value:
  *   0 (OK) or -1 (ERROR) if test failed
  *
  ****************************************************************************/
 
 static int run_cancellation_test(void)
 {
     int ret;
     int i;
     pthread_t canceler_thread;
     struct mq_attr attr;
     mqd_t mqdes;
     struct thread_data_s thread_data[NUM_THREADS];
     
     /* Create message queue */
     attr.mq_maxmsg = 10;
     attr.mq_msgsize = MESSAGE_SIZE;
     attr.mq_flags = 0;
     
     /* Unlink the message queue from any previous test run */
     mq_unlink(MQ_NAME);
     
     /* Create the message queue */
     mqdes = mq_open(MQ_NAME, O_CREAT | O_RDWR, 0666, &attr);
     if (mqdes == (mqd_t)-1) {
         printf("Error opening message queue: %d\n", errno);
         return -1;
     }
     
     g_test_counter++;
     
     /* Set up the thread data and create threads */
     for (i = 0; i < NUM_THREADS; i++) {
         thread_data[i].thread_id = i;
         thread_data[i].mqdes = mqdes;
         thread_data[i].cancel_after_ms = 100 + (rand() % 300); /* 100-400ms */
         thread_data[i].status = 0; /* running */
         
         /* Create half receiver threads and half sender threads */
         if (i % 2 == 0) {
             ret = pthread_create(&thread_data[i].thread, NULL, mq_receiver_thread, &thread_data[i]);
         } else {
             ret = pthread_create(&thread_data[i].thread, NULL, mq_sender_thread, &thread_data[i]);
         }
         
         if (ret != 0) {
             printf("Error creating thread %d: %d\n", i, ret);
             mq_close(mqdes);
             mq_unlink(MQ_NAME);
             return -1;
         }
     }
     
     /* Create canceler thread */
     ret = pthread_create(&canceler_thread, NULL, thread_canceler, thread_data);
     if (ret != 0) {
         printf("Error creating canceler thread: %d\n", ret);
         mq_close(mqdes);
         mq_unlink(MQ_NAME);
         return -1;
     }
     
     /* Wait for canceler thread to complete */
     pthread_join(canceler_thread, NULL);
     
     /* Wait for any remaining threads and clean up */
     for (i = 0; i < NUM_THREADS; i++) {
         if (thread_data[i].status == 0) { /* still running */
             pthread_join(thread_data[i].thread, NULL);
         }
     }
     
     /* Print test results */
     print_test_results(thread_data, NUM_THREADS);
     
     /* Clean up resources */
     mq_close(mqdes);
     mq_unlink(MQ_NAME);
     
     return 0;
 }
 
 /****************************************************************************
  * Name: mq_receiver_thread
  *
  * Description:
  *   A thread that waits to receive messages from the message queue.
  *   This thread will be canceled while it's blocked waiting for messages.
  *
  ****************************************************************************/
 
 static void *mq_receiver_thread(void *param)
 {
     struct thread_data_s *thread_data = (struct thread_data_s *)param;
     char buffer[MESSAGE_SIZE];
     unsigned int prio;
     int msg_count = 0;
     int result;
     
     printf("Receiver thread %d starting\n", thread_data->thread_id);
     
     /* Install cleanup handler */
     pthread_cleanup_push(cleanup_handler, thread_data);
     
     /* Make thread cancelable */
     pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
     pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
     
     /* Loop receiving messages until canceled or completed */
     while (msg_count < NUM_MESSAGES) {
         /* Wait to receive a message - this is a cancellation point */
         printf("Thread %d waiting for message...\n", thread_data->thread_id);
         result = mq_receive(thread_data->mqdes, buffer, MESSAGE_SIZE, &prio);
         
         /* Check if we were interrupted or received a message */
         if (result >= 0) {
             printf("Thread %d received message: %s\n", thread_data->thread_id, buffer);
             msg_count++;
         } else if (errno != EINTR) {
             printf("Thread %d mq_receive error: %d\n", thread_data->thread_id, errno);
             break;
         }
     }
     
     /* Mark thread as completed normally */
     pthread_mutex_lock(&g_mutex);
     thread_data->status = 2; /* completed */
     pthread_mutex_unlock(&g_mutex);
     
     /* Execute cleanup handler on thread exit */
     pthread_cleanup_pop(1);
     return NULL;
 }
 
 /****************************************************************************
  * Name: mq_sender_thread
  *
  * Description:
  *   A thread that sends messages to the message queue.
  *   This thread may get blocked if the queue is full and then be canceled.
  *
  ****************************************************************************/
 
 static void *mq_sender_thread(void *param)
 {
     struct thread_data_s *thread_data = (struct thread_data_s *)param;
     char buffer[MESSAGE_SIZE];
     int msg_count = 0;
     int result;
     
     printf("Sender thread %d starting\n", thread_data->thread_id);
     
     /* Install cleanup handler */
     pthread_cleanup_push(cleanup_handler, thread_data);
     
     /* Make thread cancelable */
     pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
     pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
     
     /* Loop sending messages until canceled or completed */
     while (msg_count < NUM_MESSAGES) {
         /* Prepare message */
         snprintf(buffer, MESSAGE_SIZE, "Message %d from thread %d", 
                  msg_count, thread_data->thread_id);
         
         /* Send message - this is a cancellation point if queue is full */
         printf("Thread %d sending message: %s\n", thread_data->thread_id, buffer);
         result = mq_send(thread_data->mqdes, buffer, strlen(buffer) + 1, 1);
         
         /* Check if we were interrupted or sent the message */
         if (result == 0) {
             printf("Thread %d sent message successfully\n", thread_data->thread_id);
             msg_count++;
             
             /* Sleep a bit to give receivers a chance */
             usleep(50 * 1000); /* 50ms */
         } else if (errno != EINTR) {
             printf("Thread %d mq_send error: %d\n", thread_data->thread_id, errno);
             break;
         }
     }
     
     /* Mark thread as completed normally */
     pthread_mutex_lock(&g_mutex);
     thread_data->status = 2; /* completed */
     pthread_mutex_unlock(&g_mutex);
     
     /* Execute cleanup handler on thread exit */
     pthread_cleanup_pop(1);
     return NULL;
 }
 
 /****************************************************************************
  * Name: thread_canceler
  *
  * Description:
  *   A thread that cancels other threads that are waiting on message queues.
  *
  ****************************************************************************/
 
 static void *thread_canceler(void *param)
 {
     struct thread_data_s *thread_data = (struct thread_data_s *)param;
     int i;
     
     printf("Canceler thread starting\n");
     
     /* Sleep to allow threads to start and block on message queue operations */
     usleep(200 * 1000); /* 200ms */
     
     /* Cancel threads in random order with different timing */
     for (i = 0; i < NUM_THREADS; i++) {
         /* Sleep for the specified time before canceling */
         usleep(thread_data[i].cancel_after_ms * 1000);
         
         /* Cancel the thread */
         printf("Canceling thread %d\n", thread_data[i].thread_id);
         if (pthread_cancel(thread_data[i].thread) != 0) {
             printf("Error canceling thread %d\n", thread_data[i].thread_id);
         }
         
         /* Wait for the thread to terminate */
         pthread_join(thread_data[i].thread, NULL);
         
         /* Check if another thread can properly use the message queue */
         if (i < NUM_THREADS - 1) {
             char test_msg[MESSAGE_SIZE];
             snprintf(test_msg, MESSAGE_SIZE, "Test message after cancel %d", i);
             
             /* Try to send a message to test if the queue is still functioning */
             if (mq_send(thread_data[0].mqdes, test_msg, strlen(test_msg) + 1, 1) == 0) {
                 printf("Successfully sent test message after cancellation %d\n", i);
             } else {
                 printf("Failed to send test message after cancellation %d: %d\n", i, errno);
             }
         }
     }
     
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

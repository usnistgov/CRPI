/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*
  win32_ulapi.c

  Implementation of user-level API to Windows simulations of the
  realtime part, plus some non-realtime resources like threads
  and sockets for portability.

  This assumes Windows user <-> Windows realtime.
*/

/* turn off sprintf warnings */
#ifndef _CRT_SECURE_NO_DEPRECATE
#define _CRT_SECURE_NO_DEPRECATE
#endif

/* Windows XP */
#define _WIN32_WINNT 0x501

#include <stdio.h>    /* printf */
#include <stddef.h>    /* NULL, sizeof */
#include <stdlib.h>    /* malloc, free */
#include <ctype.h>    /* isspace */
#include <string.h>    /* strcpy, strrchr */
#include <time.h>    /* clock, CLK_TCK */
#include <windows.h>
#include <winbase.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "ulapi.h"

static ulapi_integer ulapi_debug_level = 0;

ulapi_result ulapi_init(void)
{
  return ULAPI_OK;
}

ulapi_result ulapi_exit(void)
{
  return ULAPI_OK;
}

#define PERROR(x) if (ulapi_debug_level & ULAPI_DEBUG_ERROR) perror(x)

/*
  From http://stackoverflow.com/questions/5404277/
  porting-clock-gettime-to-windows
*/

static LARGE_INTEGER getFILETIMEoffset(void)
{
  SYSTEMTIME s;
  FILETIME f;
  LARGE_INTEGER t;

  /* base our time from the Unix epoch */
  s.wYear = 1970;
  s.wMonth = 1;
  s.wDay = 1;
  s.wHour = 0;
  s.wMinute = 0;
  s.wSecond = 0;
  s.wMilliseconds = 0;

  SystemTimeToFileTime(&s, &f);

  t.QuadPart = f.dwHighDateTime;
  t.QuadPart <<= 32;
  t.QuadPart |= f.dwLowDateTime;

  return t;
}

static int clock_gettime(struct timeval *tv)
{
  LARGE_INTEGER t;
  FILETIME f;
  double microseconds;
  static LARGE_INTEGER offset;
  static double frequencyToMicroseconds;
  static int initialized = 0;
  static BOOL usePerformanceCounter = 0;

  if (! initialized) {
    LARGE_INTEGER performanceFrequency;
    initialized = 1;
    usePerformanceCounter = QueryPerformanceFrequency(&performanceFrequency);
    if (usePerformanceCounter) {
      QueryPerformanceCounter(&offset);
      frequencyToMicroseconds = (double)performanceFrequency.QuadPart / 1000000.;
    } else {
      offset = getFILETIMEoffset();
      frequencyToMicroseconds = 10.;
    }
  }
  if (usePerformanceCounter) {
    QueryPerformanceCounter(&t);
  } else {
    GetSystemTimeAsFileTime(&f);
    t.QuadPart = f.dwHighDateTime;
    t.QuadPart <<= 32;
    t.QuadPart |= f.dwLowDateTime;
  }

  t.QuadPart -= offset.QuadPart;
  microseconds = (double) t.QuadPart / frequencyToMicroseconds;
  t.QuadPart = (LONGLONG) microseconds;
  tv->tv_sec = (long) t.QuadPart / 1000000;
  tv->tv_usec = t.QuadPart % 1000000;
  return 0;
}

ulapi_real ulapi_time(void)
{
  struct timeval tv;

  clock_gettime(&tv);

  return (ulapi_real) (((double) tv.tv_sec) + ((double) tv.tv_usec) * 1.0e-6);
}

void ulapi_sleep(ulapi_real secs)
{
  DWORD dwMilliseconds;

  /* round to nearest msec */
  dwMilliseconds = (DWORD) (secs * 1000.0 + 0.5);

  Sleep(dwMilliseconds);
}

ulapi_result ulapi_app_wait(void)
{
  /* sort of a kludge -- keep waiting a bunch of seconds */
  while (1) {
    ulapi_sleep(1000);
  }
}

ulapi_integer ulapi_to_argv(const char *src, char ***argv)
{
  char *cpy;
  char *ptr;
  ulapi_integer count;
  ulapi_flag inquote;

  *argv = NULL;
  cpy = malloc(strlen(src) + 1);
  count = 0;
  inquote = 0;

  for (; ; count++) {
    while (isspace(*src)) src++;
    if (0 == *src) return count;
    ptr = cpy;
    while (0 != *src) {
      if ('"' == *src) {
  inquote = !inquote;
      } else if ((!inquote) && isspace(*src)) {
  break;
      } else {
  *ptr++ = *src;
      }
      src++;
    }
    *ptr++ = 0;
    *argv = realloc(*argv, (count+1)*sizeof(**argv));
    (*argv)[count] = malloc(strlen(cpy) + 1);
    strcpy((*argv)[count], cpy);
  }
   
  free(cpy);

  return count;
}

void ulapi_free_argv(ulapi_integer argc, char **argv)
{
  ulapi_integer t;

  for (t = 0; t < argc; t++) {
    free(argv[t]);
  }
  free(argv);

  return;
}

void ulapi_set_debug(ulapi_integer mask)
{
  ulapi_debug_level = mask;
}

ulapi_integer ulapi_get_debug(void)
{
  return ulapi_debug_level;
}

ulapi_prio ulapi_prio_highest(void)
{
  return THREAD_PRIORITY_TIME_CRITICAL;
}

/* these are copied from win32_rtapi.c */

ulapi_prio ulapi_prio_lowest(void)
{
  /* we won't use idle priority */
  return THREAD_PRIORITY_LOWEST;
}

ulapi_prio ulapi_prio_next_higher(ulapi_prio prio)
{
  int newprio;

  switch (prio) {
  case THREAD_PRIORITY_IDLE:
  case THREAD_PRIORITY_LOWEST:
    newprio = THREAD_PRIORITY_BELOW_NORMAL;
    break;
  case THREAD_PRIORITY_BELOW_NORMAL:
    newprio = THREAD_PRIORITY_NORMAL;
    break;
  case THREAD_PRIORITY_NORMAL:
    newprio = THREAD_PRIORITY_ABOVE_NORMAL;
    break;
  case THREAD_PRIORITY_ABOVE_NORMAL:
    newprio = THREAD_PRIORITY_HIGHEST;
    break;
  case THREAD_PRIORITY_HIGHEST:
    newprio = THREAD_PRIORITY_TIME_CRITICAL;
    break;
  default:
    newprio = prio;
    break;
  }

  return newprio;
}

ulapi_prio ulapi_prio_next_lower(ulapi_prio prio)
{
  int newprio;

  switch (prio) {
  case THREAD_PRIORITY_TIME_CRITICAL:
    newprio = THREAD_PRIORITY_HIGHEST;
    break;
  case THREAD_PRIORITY_HIGHEST:
    newprio = THREAD_PRIORITY_ABOVE_NORMAL;
    break;
  case THREAD_PRIORITY_ABOVE_NORMAL:
    newprio = THREAD_PRIORITY_NORMAL;
    break;
  case THREAD_PRIORITY_NORMAL:
    newprio = THREAD_PRIORITY_BELOW_NORMAL;
    break;
  case THREAD_PRIORITY_BELOW_NORMAL:
    newprio = THREAD_PRIORITY_LOWEST;
    break;
  default:
    newprio = prio;
    break;
  }

  return newprio;
}

ulapi_result ulapi_task_init(ulapi_task_struct *task)
{
  if (NULL == task) return ULAPI_ERROR;

  task->hThread = NULL;
  task->dwThreadId = 0;

  return ULAPI_OK;
}

ulapi_task_struct *ulapi_task_new(void)
{
  ulapi_task_struct *ts;

  ts = (ulapi_task_struct *) malloc(sizeof(ulapi_task_struct));

  if (ULAPI_OK != ulapi_task_init(ts)) {
    free(ts);
    ts = NULL;
  }

  return ts;
}

ulapi_result ulapi_task_clear(ulapi_task_struct *task)
{
  if (NULL != task) {
    if (NULL != task->hThread) {
      CloseHandle(task->hThread);
    }
  }
  
  return ULAPI_OK;
}

ulapi_result ulapi_task_delete(ulapi_task_struct *task)
{
  if (NULL != task) {
    (void) ulapi_task_clear(task);
    free(task);
  }

  return ULAPI_OK;
}

ulapi_result ulapi_task_start(ulapi_task_struct *task,
     void (*taskcode)(void *),
     void *taskarg,
     ulapi_prio prio,
     ulapi_integer period_nsec)
{
  ulapi_task_struct * ts = task;
  DWORD dwCreationFlags;

  if (NULL == ts) return ULAPI_ERROR;

  dwCreationFlags = CREATE_SUSPENDED;

  ts->hThread =
    CreateThread(NULL,      /* NULL for default security attributes */
     0,  /* 0 gives default stack size */
     (LPTHREAD_START_ROUTINE) taskcode,  /* thread function */
     taskarg,  /* argument to thread function */
     dwCreationFlags, /* use default creation flags */
     &ts->dwThreadId); /* returns the thread identifier */
 
  if (NULL == ts->hThread) {
    printf("can't create thread: code %d\n", GetLastError());
    return ULAPI_ERROR;
  }

  if (0 == SetThreadPriority(ts->hThread, (int) prio)) {
    printf("can't set priority to %d: code %d\n", (int) prio, GetLastError());
    (void) ulapi_task_delete(ts);
    return ULAPI_ERROR;
  }

  if (-1 == ResumeThread(ts->hThread)) {
    printf("can't start thread: code %d\n", GetLastError());
    (void) ulapi_task_delete(ts);
    return ULAPI_ERROR;
  }

  return ULAPI_OK;
}

ulapi_result ulapi_task_stop(ulapi_task_struct *task)
{
  return ulapi_task_pause(task);
}

ulapi_result ulapi_task_pause(ulapi_task_struct *task)
{
  if (-1 == SuspendThread(task->hThread)) {
    return ULAPI_ERROR;
  }
  return ULAPI_OK;
}

ulapi_result ulapi_task_resume(ulapi_task_struct *task)
{
  if (-1 == ResumeThread(task->hThread)) {
    return ULAPI_ERROR;
  }
  return ULAPI_OK;
}

ulapi_result ulapi_task_set_period(ulapi_task_struct *task, ulapi_integer period_nsec)
{
  /* nothing to do in the task */
  return ULAPI_OK;
}

ulapi_result ulapi_self_set_period(ulapi_integer period_nsec)
{
  /* nothing can be done in the task thread */
  return ULAPI_OK;
}

ulapi_result ulapi_wait(ulapi_integer period_nsec)
{
  DWORD dwMilliseconds;

  dwMilliseconds = period_nsec / 1000000;

  Sleep(dwMilliseconds);

  return ULAPI_OK;
}

void ulapi_task_exit(ulapi_integer retval)
{
  ExitThread(retval);
}

ulapi_result ulapi_task_join(ulapi_task_struct *task, ulapi_integer *retptr)
{
  DWORD dw;
  int ret;

  if (WAIT_FAILED == WaitForSingleObject(task->hThread, INFINITE)) {
    return ULAPI_ERROR;
  }

  if (NULL != retptr) {
  ret = GetExitCodeThread(task->hThread, &dw);
    if (0 == ret) {
      return ULAPI_ERROR;
    }
    *retptr = (ulapi_integer) dw;
  }

  return ULAPI_OK;
}

ulapi_integer ulapi_task_id(void)
{
  return (ulapi_integer) GetCurrentThreadId();
}

typedef struct {
  HANDLE hProcess;
  HANDLE hThread;
} win32_process_struct;

void *ulapi_process_new(void)
{
  win32_process_struct *proc;

  proc = (win32_process_struct *) malloc(sizeof(win32_process_struct));
  if (NULL != proc) {
    proc->hProcess = NULL;
  proc->hThread = NULL;
  }

  return proc;
}

ulapi_result ulapi_process_delete(void *proc)
{
  if (NULL != proc) {
    if (NULL != ((win32_process_struct *) proc)->hProcess) {
      CloseHandle(((win32_process_struct *) proc)->hProcess);
    }
    if (NULL != ((win32_process_struct *) proc)->hThread) {
      CloseHandle(((win32_process_struct *) proc)->hThread);
    }
    free(proc);
  }

  return ULAPI_OK;
}

ulapi_result ulapi_process_start(void *proc, char *path)
{
  STARTUPINFO si;
  PROCESS_INFORMATION pi;
 
    ZeroMemory( &si, sizeof(si) );
    si.cb = sizeof(si);
    ZeroMemory( &pi, sizeof(pi) );

  
  if (! CreateProcess(NULL,
       path,
       NULL,
       NULL,
       FALSE,
       0,
       NULL,
       NULL,
       &si,
       &pi)) {
         printf( "CreateProcess failed (%d) for %s\n", GetLastError(), path);
    return ULAPI_ERROR;
  }

  ((win32_process_struct *) proc)->hProcess = pi.hProcess;
  ((win32_process_struct *) proc)->hThread = pi.hThread;

  return ULAPI_OK;
}

ulapi_result ulapi_process_stop(void *proc)
{
  int retval;
 
  retval = TerminateProcess(((win32_process_struct *) proc)->hProcess, 1);
  
  return retval ? ULAPI_OK : ULAPI_ERROR;
}

ulapi_integer ulapi_process_done(void *proc, ulapi_integer *result)
{
  DWORD retval;
  DWORD exitCode;

  retval = WaitForSingleObject(((win32_process_struct *) proc)->hProcess, 0);

  /* returns WAIT_TIMEOUT if the process is still there, else WAIT_FAILED */
  if (WAIT_TIMEOUT == retval) return 0;

  if (NULL != result) {
    if (GetExitCodeProcess(((win32_process_struct *) proc)->hProcess, &exitCode)) {
    *result = exitCode;
   } else {
    *result = -1;
   }
  }

  return 1;
}

ulapi_result ulapi_process_wait(void *proc, ulapi_integer *result)
{
  DWORD retval;
  DWORD exitCode;

  retval = WaitForSingleObject(((win32_process_struct *) proc)->hProcess, INFINITE);

  if (WAIT_OBJECT_0 == retval) {
    /* the process is done */
    if (NULL != result) {
    if (GetExitCodeProcess(((win32_process_struct *) proc)->hProcess, &exitCode)) {
      *result = exitCode;
    } else {
      *result = -1;
    }
    }
    return ULAPI_OK;
  }

  return ULAPI_ERROR;
}

ulapi_result ulapi_mutex_init(ulapi_mutex_struct *mutex, ulapi_id key)
{
  HANDLE hMutex;

  hMutex = CreateMutex(NULL,  /* default security attributes */
           FALSE,  /* initially not owned */
           NULL);  /* unnamed mutex */

  if (NULL == hMutex) {
    return ULAPI_ERROR;
  }

  mutex->hMutex = hMutex;

  return ULAPI_OK;
}

ulapi_mutex_struct *ulapi_mutex_new(ulapi_id key)
{
  ulapi_mutex_struct *mutex;
  HANDLE hMutex;

  mutex = (ulapi_mutex_struct *) malloc(sizeof(ulapi_mutex_struct));
  if (NULL == mutex) {
    return NULL;
  }

  hMutex = CreateMutex(NULL,  /* default security attributes */
           FALSE,  /* initially not owned */
           NULL);  /* unnamed mutex */

  if (NULL == hMutex) {
    free(mutex);
    return NULL;
  }

  mutex->hMutex = hMutex;

  return mutex;
}

ulapi_result ulapi_mutex_clear(ulapi_mutex_struct *mutex)
{
  CloseHandle(mutex->hMutex);

  return ULAPI_OK;
}

ulapi_result ulapi_mutex_delete(ulapi_mutex_struct *mutex)
{
  if (NULL == mutex) return ULAPI_ERROR;

  CloseHandle(mutex->hMutex);
  free(mutex);

  return ULAPI_OK;
}

ulapi_result ulapi_mutex_give(ulapi_mutex_struct *mutex)
{
  BOOL retval;

  retval = ReleaseMutex(mutex->hMutex);

  return retval ? ULAPI_OK : ULAPI_ERROR;
}

ulapi_result ulapi_mutex_take(ulapi_mutex_struct *mutex)
{
  DWORD retval;

  retval = WaitForSingleObject(mutex->hMutex, INFINITE);

  return retval == WAIT_OBJECT_0 ? ULAPI_OK : ULAPI_ERROR;
} 

/* this needs to be static, not heap or stack */
static char ulapi_sem_name[3   /* for "sem" */
         + DIGITS_IN(ulapi_id) /* for the number */
         + 1]; /* for the null */

typedef struct {
  HANDLE hSemaphore;
} win32_sem_struct;

void *ulapi_sem_new(ulapi_id key)
{
  win32_sem_struct * sem;
  HANDLE hSemaphore;

  sem = (win32_sem_struct *) malloc(sizeof(win32_sem_struct));
  if (NULL == sem) {
    return NULL;
  }

  sprintf(ulapi_sem_name, "sem%d", (int) key);

  hSemaphore = CreateSemaphore(NULL,
             1, /* initial count */
             1, /* max count */
             ulapi_sem_name);
  if (NULL == hSemaphore) {
    free(sem);
    return NULL;
  }

  sem->hSemaphore = hSemaphore;

  return sem;
}

ulapi_result ulapi_sem_delete(void *sem)
{
  if (NULL == sem) return ULAPI_ERROR;

  CloseHandle(((win32_sem_struct *) sem)->hSemaphore);
  free(sem);
  
  return ULAPI_OK;
}

ulapi_result ulapi_sem_give(void *sem)
{
  BOOL retval;

  retval = ReleaseSemaphore(((win32_sem_struct *) sem)->hSemaphore,
          1,  /* how much to give */
          NULL); /* no need for previous count */
  return retval ? ULAPI_OK : ULAPI_ERROR;
}

ulapi_result ulapi_sem_take(void *sem)
{
  DWORD retval;

  retval = WaitForSingleObject(((win32_sem_struct *) sem)->hSemaphore,
             INFINITE);

  return retval == WAIT_OBJECT_0 ? ULAPI_OK : ULAPI_ERROR;
}

/*
  See http://www.cs.wustl.edu/~schmidt/win32-cv-1.html for details
  on implementing condition variables in Win32.
*/

typedef struct {
  int waiters_count_;
  /* Number of waiting threads. */

  CRITICAL_SECTION waiters_count_lock_;
  /* Serialize access to <waiters_count_>. */

  HANDLE sema_;
  /* Semaphore used to queue up threads waiting for the condition to */
  /* become signaled.  */

  HANDLE waiters_done_;
  /* An auto-reset event used by the broadcast/signal thread to wait */
  /* for all the waiting thread(s) to wake up and be released from the */
  /* semaphore.  */

  size_t was_broadcast_;
  /* Keeps track of whether we were broadcasting or signaling.  This */
  /* allows us to optimize the code if we're just signaling. */
} pthread_cond_t;

typedef int pthread_condattr_t;

typedef HANDLE pthread_mutex_t;

static int pthread_cond_init(pthread_cond_t *cv,
           const pthread_condattr_t * attr)
{
  cv->waiters_count_ = 0;
  cv->was_broadcast_ = 0;
  cv->sema_ = CreateSemaphore(NULL,       /* no security */
            0,          /* initially 0 */
            0x7fffffff, /* max count */
            NULL);      /* unnamed  */
  InitializeCriticalSection (&cv->waiters_count_lock_);
  cv->waiters_done_ = CreateEvent(NULL,  /* no security */
          FALSE, /* auto-reset */
          FALSE, /* non-signaled initially */
          NULL); /* unnamed */

  return 0;
}

static int pthread_cond_wait(pthread_cond_t *cv, 
           pthread_mutex_t *external_mutex)
{
  int last_waiter;

  /* Avoid race conditions. */
  EnterCriticalSection (&cv->waiters_count_lock_);
  cv->waiters_count_++;
  LeaveCriticalSection (&cv->waiters_count_lock_);

  /* This call atomically releases the mutex and waits on the */
  /* semaphore until <pthread_cond_signal> or <pthread_cond_broadcast> */
  /* are called by another thread. */
  SignalObjectAndWait (*external_mutex, cv->sema_, INFINITE, FALSE);

  /* Reacquire lock to avoid race conditions. */
  EnterCriticalSection (&cv->waiters_count_lock_);

  /* We're no longer waiting... */
  cv->waiters_count_--;

  /* Check to see if we're the last waiter after <pthread_cond_broadcast>. */
  last_waiter = cv->was_broadcast_ && cv->waiters_count_ == 0;

  LeaveCriticalSection (&cv->waiters_count_lock_);

  /* If we're the last waiter thread during this particular broadcast */
  /* then let all the other threads proceed. */
  if (last_waiter)
    /* This call atomically signals the <waiters_done_> event and waits until */
    /* it can acquire the <external_mutex>.  This is required to ensure fairness.  */
    SignalObjectAndWait (cv->waiters_done_, *external_mutex, INFINITE, FALSE);
  else
    /* Always regain the external mutex since that's the guarantee we */
    /* give to our callers.  */
    WaitForSingleObject (*external_mutex, INFINITE);

  return 0;
}

static int pthread_cond_signal(pthread_cond_t * cv)
{
  int have_waiters;

  EnterCriticalSection (&cv->waiters_count_lock_);
  have_waiters = cv->waiters_count_ > 0;
  LeaveCriticalSection (&cv->waiters_count_lock_);

  /* If there aren't any waiters, then this is a no-op.   */
  if (have_waiters)
    ReleaseSemaphore (cv->sema_, 1, 0);

  return 0;
}

static int pthread_cond_broadcast(pthread_cond_t * cv)
{
  int have_waiters = 0;

  /* This is needed to ensure that <waiters_count_> and <was_broadcast_> are */
  /* consistent relative to each other. */
  EnterCriticalSection (&cv->waiters_count_lock_);

  if (cv->waiters_count_ > 0) {
    /* We are broadcasting, even if there is just one waiter... */
    /* Record that we are broadcasting, which helps optimize */
    /* <pthread_cond_wait> for the non-broadcast case. */
    cv->was_broadcast_ = 1;
    have_waiters = 1;
  }

  if (have_waiters) {
    /* Wake up all the waiters atomically. */
    ReleaseSemaphore (cv->sema_, cv->waiters_count_, 0);

    LeaveCriticalSection (&cv->waiters_count_lock_);

    /* Wait for all the awakened threads to acquire the counting */
    /* semaphore.  */
    WaitForSingleObject (cv->waiters_done_, INFINITE);
    /* This assignment is okay, even without the <waiters_count_lock_> held  */
    /* because no other waiter threads can wake up to access it. */
    cv->was_broadcast_ = 0;
  }
  else
    LeaveCriticalSection (&cv->waiters_count_lock_);

  return 0;
}

static void pthread_cond_destroy(pthread_cond_t * cond)
{
  return;
}

void *ulapi_cond_new(ulapi_id key)
{
  pthread_cond_t * cond;

  cond = (pthread_cond_t *) malloc(sizeof(pthread_cond_t));
  if (NULL == (void *) cond) return NULL;

  if (0 == pthread_cond_init(cond, NULL)) {
    return (void *) cond;
  }
  /* else got an error, so free the condition variable and return null */

  free(cond);
  return NULL;
}

ulapi_result ulapi_cond_delete(void *cond)
{
  if (NULL == (void *) cond) return ULAPI_ERROR;

  (void) pthread_cond_destroy((pthread_cond_t *) cond);
  free(cond);

  return ULAPI_OK;
}

ulapi_result ulapi_cond_signal(void *cond)
{
  return (0 == pthread_cond_signal((pthread_cond_t *) cond) ? ULAPI_OK : ULAPI_ERROR);
}

ulapi_result ulapi_cond_broadcast(void *cond)
{
  return (0 == pthread_cond_broadcast((pthread_cond_t *) cond) ? ULAPI_OK : ULAPI_ERROR);
}

ulapi_result ulapi_cond_wait(void *cond, void *mutex)
{
  return (0 == pthread_cond_wait((pthread_cond_t *) cond, (pthread_mutex_t *) mutex) ? ULAPI_OK : ULAPI_ERROR);
}

/* this needs to be static, not heap or stack */
static char ulapi_shm_name[3   /* for "shm" */
         + DIGITS_IN(ulapi_id) /* for the number */
         + 1]; /* for the null */

typedef struct {
  HANDLE hMapFile;
  void *ptr;
} win32_shm_struct;

void *ulapi_shm_new(ulapi_id key, ulapi_integer size)
{
  win32_shm_struct * shm;
  HANDLE hMapFile;
  void *ptr;

  shm = (win32_shm_struct *) malloc(sizeof(win32_shm_struct));
  if (NULL == shm) {
    return NULL;
  }

  ulapi_snprintf(ulapi_shm_name, sizeof(ulapi_shm_name), "shm%d", (int) key);
  ulapi_shm_name[sizeof(ulapi_shm_name)-1] = 0;

  hMapFile = 
    CreateFileMapping(INVALID_HANDLE_VALUE, /* use paging file */
          NULL,  /* default security  */
          PAGE_READWRITE, /* read/write access */
          0,  /* max. object size  */
          size,  /* buffer size */
          ulapi_shm_name); /* name of mapping object */
 
  if (hMapFile == NULL || hMapFile == INVALID_HANDLE_VALUE) {
    free(shm);
    return NULL;
  }

  ptr = MapViewOfFile(hMapFile,   /* handle to map object */
          FILE_MAP_ALL_ACCESS, /* read/write permission */
          0,                   
          0,                   
          size);           
 
  if (ptr == NULL) {
    free(shm);
    return NULL;
  }

  shm->hMapFile = hMapFile;
  shm->ptr = ptr;

  return shm;
}

void *ulapi_shm_addr(void *shm)
{
  if (NULL == shm) return NULL;

  return ((win32_shm_struct *) shm)->ptr;
}

ulapi_result ulapi_shm_delete(void *shm)
{
  if (NULL == shm) return ULAPI_ERROR;

  UnmapViewOfFile(((win32_shm_struct *) shm)->ptr);
  CloseHandle(((win32_shm_struct *) shm)->hMapFile);

  free(shm);

  return ULAPI_OK;
}

void *ulapi_rtm_new(ulapi_id key, ulapi_integer size)
{
  return ulapi_shm_new(key, size);
}

void *ulapi_rtm_addr(void * shm)
{
  return ulapi_shm_addr(shm);
}

ulapi_result ulapi_rtm_delete(void * shm)
{
  return ulapi_shm_delete(shm);
}

char *ulapi_address_to_hostname(ulapi_integer address)
{
  static char string[4 * DIGITS_IN(int) + 3 + 1];

  sprintf(string, "%d.%d.%d.%d",
    (int) ((address >> 24) & 0xFF),
    (int) ((address >> 16) & 0xFF),
    (int) ((address >> 8) & 0xFF),
    (int) (address & 0xFF));

  return string;
}

/* this is equivalent to inet_addr() */
ulapi_integer ulapi_hostname_to_address(const char *hostname)
{
  struct hostent *ent;

  ent = gethostbyname(hostname);
  if (NULL == ent) return (127 << 24) + (0 << 16) + (0 << 8) + 1;
  if (4 != ent->h_length) return 0;

  /* FIXME-- could use ntohl here */
  return ((ent->h_addr_list[0][0] & 0xFF) << 24)
    + ((ent->h_addr_list[0][1] & 0xFF) << 16)
    + ((ent->h_addr_list[0][2] & 0xFF) << 8)
    + (ent->h_addr_list[0][3] & 0xFF);
}

ulapi_integer ulapi_get_host_address(void)
{
  WSADATA wsaData;
  int iResult;
  enum {HOSTNAMELEN = 256};
  char hostname[HOSTNAMELEN];
  ulapi_integer retval;

  iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
  if (iResult != 0) {
    return 0;
  }

  retval = gethostname(hostname, HOSTNAMELEN);
  if (0 != retval) {
  return 0;
  }
  hostname[HOSTNAMELEN - 1] = 0; /* force null termination */

  return ulapi_hostname_to_address(hostname);
}

ulapi_integer ulapi_socket_get_client_id(ulapi_integer port,
           const char * hostname)
{
  WSADATA wsaData;
  int iResult;
  int socket_fd;
  struct sockaddr_in server_addr;
  struct hostent *ent;
  struct in_addr *in_a;

  iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
  if (iResult != 0) {
    return -1;
  }

  if (NULL == (ent = gethostbyname(hostname))) {
    PERROR("gethostbyname");
    return -1;
  }
  in_a = (struct in_addr *) ent->h_addr_list[0];

  memset(&server_addr, 0, sizeof(struct sockaddr_in));
  server_addr.sin_family = PF_INET;
  server_addr.sin_addr.s_addr = in_a->s_addr;
  server_addr.sin_port = htons(port);

  if (-1 == (socket_fd = socket(PF_INET, SOCK_STREAM, 0))) {
    PERROR("socket");
    return -1;
  }

  if (-1 == connect(socket_fd, 
        (struct sockaddr *) &server_addr,
        sizeof(struct sockaddr_in))) {
    PERROR("connect");
    closesocket(socket_fd);
    return -1;
  }

  return socket_fd;
}

ulapi_integer ulapi_socket_get_server_id_on_interface(ulapi_integer port, const char *intf)
{
  WSADATA wsaData;
  int iResult;
  int socket_fd;
  struct sockaddr_in myaddr;
  int on;
  struct linger mylinger = { 0 };
  enum {BACKLOG = 5};

  iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
  if (iResult != 0) {
    return -1;
  }

  if (-1 == (socket_fd = socket(PF_INET, SOCK_STREAM, 0))) {
    PERROR("socket");
    return -1;
  }

  /*
    Turn off bind address checking, and allow
    port numbers to be reused - otherwise the
    TIME_WAIT phenomenon will prevent binding
    to these address.port combinations for
    (2 * MSL) seconds.
  */
  on = 1;
  if (-1 == 
      setsockopt(socket_fd,
     SOL_SOCKET,
     SO_REUSEADDR,
     (const char *) &on,
     sizeof(on))) {
    PERROR("setsockopt");
    closesocket(socket_fd);
    return -1;
  }

  /*
    When connection is closed, there is a need
    to linger to ensure all data is transmitted.
  */
  mylinger.l_onoff = 1;
  mylinger.l_linger = 30;
  if (-1 ==
      setsockopt(socket_fd,
     SOL_SOCKET,
     SO_LINGER,
     (const char *) &mylinger,
     sizeof(struct linger))) {
    PERROR("setsockopt");
    closesocket(socket_fd);
    return -1;
  }

  memset(&myaddr, 0, sizeof(struct sockaddr_in));
  myaddr.sin_family = PF_INET;
  if (NULL == intf) {
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else {
    myaddr.sin_addr.s_addr = inet_addr(intf);
  }
  myaddr.sin_port = htons(port);

  if (-1 == bind(socket_fd, (struct sockaddr *) &myaddr,
     sizeof(struct sockaddr_in))) {
    PERROR("bind");
    closesocket(socket_fd);
    return -1;
  }

  if (-1 == listen(socket_fd, BACKLOG)) {
    PERROR("listen");
    closesocket(socket_fd);
    return -1;
  }

  return socket_fd;
}

ulapi_integer ulapi_socket_get_server_id(ulapi_integer port)
{
  return ulapi_socket_get_server_id_on_interface(port, NULL);
}

ulapi_integer ulapi_socket_get_connection_id(ulapi_integer socket_fd)
{
  struct sockaddr_in client_addr;
  int client_len;
  int client_fd;

  memset(&client_addr, 0, sizeof(struct sockaddr_in));
  client_len = sizeof(struct sockaddr_in);
  client_fd = 
    accept(socket_fd,
     (struct sockaddr *) &client_addr, 
     &client_len);
  if (-1 == client_fd) {
    PERROR("accept");
    return -1;
  }
  
  return client_fd;
}

ulapi_integer ulapi_socket_get_broadcaster_id_on_interface(ulapi_integer port, const char *intf)
{
  WSADATA wsaData;
  int iResult;
  struct sockaddr_in addr;
  int fd;
  int perm;

  iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
  if (iResult != 0) {
    return -1;
  }

  fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (0 > fd) {
    return -1;
  }

  perm = 1;
  if (0 > setsockopt(fd, SOL_SOCKET, SO_BROADCAST, (void *) &perm, sizeof(perm))) {
    return -1;
  }

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  if (NULL == intf) {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else {
    addr.sin_addr.s_addr = inet_addr(intf);
  }
  addr.sin_port = htons(port);

  if (-1 == connect(fd, 
        (struct sockaddr *) &addr,
        sizeof(struct sockaddr_in))) {
    PERROR("connect");
    closesocket(fd);
    return -1;
  }

  return fd;
}

ulapi_integer
ulapi_socket_get_broadcaster_id(ulapi_integer port)
{
  return ulapi_socket_get_broadcaster_id_on_interface(port, NULL);
}

ulapi_integer ulapi_socket_get_broadcastee_id_on_interface(ulapi_integer port, const char *intf)
{
  WSADATA wsaData;
  int iResult;
  struct sockaddr_in addr;
  int fd;
  int retval;

  iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
  if (iResult != 0) {
    return -1;
  }

  /* Create a best-effort datagram socket using UDP */
  fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (0 > fd) {
    return -1;
  }

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  if (NULL == intf) {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else {
    addr.sin_addr.s_addr = inet_addr(intf);
  }
  addr.sin_port = htons(port);

  retval = bind(fd, (struct sockaddr *) &addr, sizeof(addr));

  if (0 > retval) {
    closesocket(fd);
    return -1;
  }

  return fd;
}

ulapi_integer ulapi_socket_get_broadcastee_id(ulapi_integer port)
{
  return ulapi_socket_get_broadcastee_id_on_interface(port, NULL);
}

ulapi_result ulapi_socket_set_nonblocking(ulapi_integer fd)
{
  /* FIXME-- implement this */
  return ULAPI_ERROR;
}

ulapi_result ulapi_socket_set_blocking(ulapi_integer fd)
{
  /* FIXME-- implement this */
  return ULAPI_ERROR;
}

ulapi_integer ulapi_socket_read(ulapi_integer id,
        char *buf,
        ulapi_integer len)
{
  return recv(id, buf, len, 0);
}

ulapi_integer ulapi_socket_write(ulapi_integer id,
         const char *buf,
         ulapi_integer len)
{
  return send(id, buf, len, 0);
}

ulapi_result ulapi_socket_close(ulapi_integer id)
{
  return 0 == closesocket((int) id) ? ULAPI_OK : ULAPI_ERROR;
}

/* File descriptor interface */

void *ulapi_fd_new(void)
{
  return malloc(sizeof(HANDLE));
}

ulapi_result ulapi_fd_delete(void *id)
{
  if (NULL != id) free(id);

  return ULAPI_OK;
}

ulapi_result ulapi_std_open(ulapi_stdio io, void *id)
{
  if (ULAPI_STDIN == io) {
    *((HANDLE *) id) = GetStdHandle(STD_INPUT_HANDLE);
    return ULAPI_OK;
  }
  if (ULAPI_STDOUT == io) {
    *((HANDLE *) id) = GetStdHandle(STD_OUTPUT_HANDLE);
    return ULAPI_OK;
  }
  if (ULAPI_STDERR == io) {
    *((HANDLE *) id) = GetStdHandle(STD_ERROR_HANDLE);
    return ULAPI_OK;
  }
  return ULAPI_ERROR;
}

ulapi_result ulapi_fd_stat(const char *path)
{
  struct _stat buf;

  return 0 == _stat(path, &buf) ? ULAPI_OK : ULAPI_ERROR;
  
}

ulapi_result ulapi_fd_open(const char *path, void *id)
{
  HANDLE handle;

  handle = CreateFile(path,
          GENERIC_READ | GENERIC_WRITE, 
          0, 
          NULL, 
          OPEN_EXISTING, 
          0, 
          NULL);

  if (handle == INVALID_HANDLE_VALUE) return ULAPI_ERROR;

  *((HANDLE *) id) = handle;
  return ULAPI_OK;
}

ulapi_result ulapi_fd_set_nonblocking(void *id)
{
  return ULAPI_OK;
}

ulapi_result ulapi_fd_set_blocking(void *id)
{
  return ULAPI_OK;
}

ulapi_integer ulapi_fd_read(void *id,
          char *buf,
          ulapi_integer len)
{
  ulapi_integer nchars;

  if (ReadFile(*((HANDLE *) id), buf, len, &nchars, NULL) != 0)
    return nchars;

  return -1;
}

ulapi_integer ulapi_fd_write(void *id,
           const char *buf,
           ulapi_integer len)
{
  ulapi_integer nchars;

  if (WriteFile(*((HANDLE *) id), buf, len, &nchars, NULL) != 0)
    return nchars;

  return -1;
}

void ulapi_fd_flush(void *id)
{
  return;
}

void ulapi_fd_drain(void *id)
{
  return;
}

ulapi_result ulapi_fd_close(void *id)
{
  CloseHandle(*((HANDLE *) id));
  
  return ULAPI_OK;
}

/* Serial interface is implemented using the file descriptor interface,
   which is the same */

void *ulapi_serial_new(void)
{
  return ulapi_fd_new();
}

ulapi_result ulapi_serial_delete(void *id)
{
  return ulapi_fd_delete(id);
}

ulapi_result ulapi_serial_open(const char *port, void *id)
{
  ulapi_result retval;
  COMMTIMEOUTS ct;

  ct.ReadIntervalTimeout = 1;
  ct.ReadTotalTimeoutMultiplier = 1;
  ct.ReadTotalTimeoutConstant = 0;
  ct.WriteTotalTimeoutMultiplier = 1;
  ct.WriteTotalTimeoutConstant = 0;

  retval = ulapi_fd_open(port, id);
  if (ULAPI_OK != retval) return retval;
  SetCommTimeouts(*((HANDLE *) id), &ct);

  return retval;
}

static int to_cbr(int baud)
{
  int cbr;

  switch (baud) {
    case 110:
      cbr = CBR_110;
      break;
    case 300:
      cbr = CBR_300;
      break;
    case 1200:
      cbr = CBR_1200;
      break;
    case 2400:
      cbr = CBR_2400;
      break;
    case 4800:
      cbr = CBR_4800;
      break;
    case 9600:
      cbr = CBR_9600;
      break;
    case 19200:
      cbr = CBR_19200;
      break;
    case 38400:
      cbr = CBR_38400;
      break;
    case 57600: 
      cbr = CBR_57600;
      break;
    case 115200:
      cbr = CBR_115200;
      break;
    default:
      cbr = CBR_115200;
      break;
  } /* switch (baud) */

  return cbr;
}

ulapi_result ulapi_serial_baud(void *id, int baud)
{
  HANDLE handle = *((HANDLE *) id);
  int cbr = to_cbr(baud);
  DCB dcb;
  COMMTIMEOUTS ct;
  int msPerByte;

  if (baud <= 0) return ULAPI_ERROR;

  dcb.DCBlength = sizeof(DCB);
  GetCommState(handle, &dcb);
  dcb.BaudRate = cbr;
  dcb.ByteSize = DATABITS_8;
  dcb.Parity = PARITY_NONE;
  dcb.StopBits = ONESTOPBIT;

  if (! SetCommState(handle, &dcb)) return ULAPI_ERROR;

  msPerByte = 9000 / baud + 1; // 8 data, 1 stop = 9
  ct.ReadIntervalTimeout = msPerByte;
  ct.ReadTotalTimeoutMultiplier = msPerByte;
  ct.ReadTotalTimeoutConstant = 0;
  ct.WriteTotalTimeoutMultiplier = msPerByte;
  ct.WriteTotalTimeoutConstant = 0;
  SetCommTimeouts(*((HANDLE *) id), &ct);

  return ULAPI_OK;
}

ulapi_result ulapi_serial_set_nonblocking(void *id)
{
  return ulapi_fd_set_nonblocking(id);
}

ulapi_result ulapi_serial_set_blocking(void *id)
{
  return ulapi_fd_set_blocking(id);
}

ulapi_integer ulapi_serial_read(void *id,
        char *buf,
        ulapi_integer len)
{
  return ulapi_fd_read(id, buf, len);
}

ulapi_integer ulapi_serial_write(void *id,
         const char *buf,
         ulapi_integer len)
{
  return ulapi_fd_write(id, buf, len);
}

ulapi_result ulapi_serial_close(void *id)
{
  return ulapi_fd_close(id);
}

void *ulapi_dl_open(const char *objname, char *errstr, int errlen)
{
  HINSTANCE hinstLib; 
 
  hinstLib = LoadLibrary(objname);
 
  if (NULL == hinstLib) {
    if (NULL != errstr) {
      strncpy(errstr, "LoadLibrary error", errlen);
      errstr[errlen - 1] = 0;
    }
    return NULL;
  }

  return hinstLib;
}

void ulapi_dl_close(void *handle)
{
  if (NULL != handle) {
    FreeLibrary(handle);
  }

  return;
}

void *ulapi_dl_sym(void *handle, const char *name, char *errstr, int errlen)
{
  DWORD lastError;
  void *ProcAdd;

  SetLastError(0);
  ProcAdd = GetProcAddress(handle, name);
  lastError = GetLastError();
  
  if (NULL == ProcAdd) {
    if (0 != lastError) {
      if (NULL != errstr) {
  FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM,
          NULL,
          lastError,
          0,
          errstr,
          errlen,
          NULL);
  errstr[errlen - 1] = 0;
      }
    } else {
      if (NULL != errstr) {
  errstr[0] = 0;
      }
    }
    return NULL;
  }

  if (NULL != errstr) {
    errstr[0] = 0;
  }

  return ProcAdd;
}

/*
  You can implicitly convert a char array to an LPCSTR without any casts:

  void SomeFunction(LPCSTR aString);
  ...
  char myArray[] = "hello, world!";
  SomeFunction(myArray);

  An LPCSTR is a Windows typedef for a long pointer to a constant
  string.  Back in the dark days of Win16 programming, there were
  different types of pointers: near pointers and far pointers,
  sometimes also known as short and long pointers respectively.  Near
  pointers could only point to a 64KB segment of memory determined by
  one of the x86 segment registers.  Far pointers could point to
  anything. Nowadays in Win32 with virtual memory, there is no need
  for near pointers -- all pointers are long.

  So, an LPSTR is a typedef for a char *, or pointer to a string. An
  LPCSTR is the const version, i.e. it is a typedef for a const char *.
  In C, arrays decay into pointers to their first elements, so a
  char[] decays into a char*. Finally, any type of "pointer to T" (for
  any type T) can be implicitly converted into a "pointer to const T".
  Thus, combining these three facts, we see that we can implicitly
  convert a char[] into an LPCSTR.
*/

ulapi_result ulapi_system(const char *prog, int *result)
{
  STARTUPINFO si;
  PROCESS_INFORMATION pi;
  DWORD retval;

  ZeroMemory(&si, sizeof(si));
  si.cb = sizeof(si);
  ZeroMemory(&pi, sizeof(pi));

  if (!CreateProcess(NULL,  // No module name (use command line)
         (LPSTR) prog,  // Command line
         NULL,  // Process handle not inheritable
         NULL,  // Thread handle not inheritable
         FALSE,  // Set handle inheritance to FALSE
         0,    // No creation flags
         NULL,  // Use parent's environment block
         NULL,  // Use parent's starting directory 
         &si,  // Pointer to STARTUPINFO structure
         &pi)) { // Pointer to PROCESS_INFORMATION structure
    return ULAPI_ERROR;
  }

  WaitForSingleObject(pi.hProcess, INFINITE);

  GetExitCodeProcess(pi.hProcess, &retval);
  *result = retval;

  /* Close process and thread handles.  */
  CloseHandle(pi.hProcess);
  CloseHandle(pi.hThread);

  return ULAPI_OK;
}

ulapi_flag ulapi_ispath(const char *path)
{
  if ((NULL == path) || 
      (NULL == strchr(path, '\\'))) {
    return 0;
  }

  return 1;
}

char *ulapi_fixpath(const char *path, char *fix, size_t len)
{
  const char *pathptr = path;
  char *fixptr = fix;
  char c;

  if (NULL == fixptr) return NULL;
  if (len <= 0) return fixptr;

  while (0 != *pathptr && --len > 0) {
    c = *pathptr++;
    if ('/' == c) c = '\\';
    *fixptr++ = c;
  }

  *fixptr = 0;

  return fix;
}

char *ulapi_basename(const char *path, char *base)
{
  const char *endp, *startp;

  if (path == NULL || *path == '\0') {
    strcpy(base, ".");
    return base;
  }

  endp = path + strlen(path) - 1;
  while (endp > path && *endp == '\\')
    endp--;

  if (endp == path && *endp == '\\') {
    strcpy(base, "\\");
    return base;
  }

  startp = endp;
  while (startp > path && *(startp - 1) != '\\') startp--;

  strcpy(base, startp);
  base[(endp - startp) + 1] = 0;

  return base;
}

char *ulapi_dirname(const char *path, char *base)
{
  const char *endp;

  if (path == NULL || *path == '\0') {
    strcpy(base, ".");
    return base;
  }

  endp = path + strlen(path) - 1;
  while (endp > path && *endp == '\\') endp--;

  while (endp > path && *endp != '\\') endp--;

  if (endp == path) {
    strcpy(base, *endp == '\\' ? "\\" : ".");
    return base;
  }

  do {
    endp--;
  } while (endp > path && *endp == '\\');

  strcpy(base, path);
  base[(endp - path) + 1] = 0;

  return base;
}

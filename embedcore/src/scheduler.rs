//! Cooperative task scheduler for EmbedCore

use super::task::{Task, TaskId, TaskPriority, TaskState, TaskResult, TaskFunction};
use std::collections::BinaryHeap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use std::thread;

/// Scheduler configuration
#[derive(Debug, Clone, Copy)]
pub struct SchedulerConfig {
    pub max_tasks: usize,
    pub tick_interval_ms: u32,
    pub enable_deadline_monitoring: bool,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        SchedulerConfig {
            max_tasks: 32,
            tick_interval_ms: 1,
            enable_deadline_monitoring: true,
        }
    }
}

/// Scheduler statistics
#[derive(Debug, Clone, Copy)]
pub struct SchedulerStats {
    pub total_tasks: usize,
    pub ready_tasks: usize,
    pub running_tasks: usize,
    pub blocked_tasks: usize,
    pub suspended_tasks: usize,
    pub completed_tasks: usize,
    pub missed_deadlines: u32,
    pub total_ticks: u64,
}

/// Task scheduler error
#[derive(Debug, thiserror::Error)]
pub enum SchedulerError {
    #[error("Maximum number of tasks exceeded")]
    MaxTasksExceeded,
    #[error("Task with ID {0} not found")]
    TaskNotFound(TaskId),
    #[error("Task with ID {0} already exists")]
    TaskAlreadyExists(TaskId),
    #[error("Scheduler is not running")]
    SchedulerNotRunning,
}

/// Cooperative task scheduler
/// 
/// Provides a simple cooperative task scheduler that runs tasks based on
/// priority and deadlines. Tasks must yield control voluntarily.
#[derive(Debug)]
pub struct Scheduler {
    tasks: Arc<Mutex<BinaryHeap<Task>>>,
    task_map: Arc<Mutex<std::collections::HashMap<TaskId, Task>>>,
    next_task_id: Arc<Mutex<TaskId>>,
    config: SchedulerConfig,
    stats: Arc<Mutex<SchedulerStats>>,
    running: Arc<Mutex<bool>>,
}

impl Scheduler {
    /// Create a new scheduler with default configuration
    pub fn new() -> Self {
        Self::with_config(SchedulerConfig::default())
    }
    
    /// Create a new scheduler with custom configuration
    pub fn with_config(config: SchedulerConfig) -> Self {
        Scheduler {
            tasks: Arc::new(Mutex::new(BinaryHeap::new())),
            task_map: Arc::new(Mutex::new(std::collections::HashMap::new())),
            next_task_id: Arc::new(Mutex::new(1)),
            config,
            stats: Arc::new(Mutex::new(SchedulerStats {
                total_tasks: 0,
                ready_tasks: 0,
                running_tasks: 0,
                blocked_tasks: 0,
                suspended_tasks: 0,
                completed_tasks: 0,
                missed_deadlines: 0,
                total_ticks: 0,
            })),
            running: Arc::new(Mutex::new(false)),
        }
    }
    
    /// Add a task to the scheduler
    /// 
    /// # Arguments
    /// * `name` - Task name for debugging
    /// * `priority` - Task priority
    /// * `function` - Task function to execute
    /// * `period_ms` - Optional period for periodic tasks
    /// 
    /// # Returns
    /// * `Result<TaskId, SchedulerError>` - Task ID if successful, error otherwise
    pub fn add_task<F>(
        &self,
        name: String,
        priority: TaskPriority,
        function: F,
        period_ms: Option<u32>,
    ) -> Result<TaskId, SchedulerError>
    where
        F: FnMut() -> TaskResult + Send + Sync + 'static,
    {
        let mut next_id = self.next_task_id.lock().unwrap();
        let task_id = *next_id;
        *next_id += 1;
        drop(next_id);
        
        let mut tasks = self.tasks.lock().unwrap();
        let mut task_map = self.task_map.lock().unwrap();
        
        if tasks.len() >= self.config.max_tasks {
            return Err(SchedulerError::MaxTasksExceeded);
        }
        
        if task_map.contains_key(&task_id) {
            return Err(SchedulerError::TaskAlreadyExists(task_id));
        }
        
        let task = Task::new(
            task_id,
            name,
            priority,
            TaskFunction::new(function),
            period_ms,
        );
        
        tasks.push(task.clone());
        task_map.insert(task_id, task);
        
        self.update_stats();
        
        Ok(task_id)
    }
    
    /// Remove a task from the scheduler
    /// 
    /// # Arguments
    /// * `task_id` - The task ID to remove
    /// 
    /// # Returns
    /// * `Result<(), SchedulerError>` - Ok if successful, error otherwise
    pub fn remove_task(&self, task_id: TaskId) -> Result<(), SchedulerError> {
        let mut task_map = self.task_map.lock().unwrap();
        
        if !task_map.contains_key(&task_id) {
            return Err(SchedulerError::TaskNotFound(task_id));
        }
        
        task_map.remove(&task_id);
        drop(task_map);
        
        // Rebuild the priority queue without the removed task
        self.rebuild_task_queue();
        self.update_stats();
        
        Ok(())
    }
    
    /// Suspend a task
    /// 
    /// # Arguments
    /// * `task_id` - The task ID to suspend
    /// 
    /// # Returns
    /// * `Result<(), SchedulerError>` - Ok if successful, error otherwise
    pub fn suspend_task(&self, task_id: TaskId) -> Result<(), SchedulerError> {
        let mut task_map = self.task_map.lock().unwrap();
        
        if let Some(task) = task_map.get_mut(&task_id) {
            task.state = TaskState::Suspended;
            self.update_stats();
            Ok(())
        } else {
            Err(SchedulerError::TaskNotFound(task_id))
        }
    }
    
    /// Resume a suspended task
    /// 
    /// # Arguments
    /// * `task_id` - The task ID to resume
    /// 
    /// # Returns
    /// * `Result<(), SchedulerError>` - Ok if successful, error otherwise
    pub fn resume_task(&self, task_id: TaskId) -> Result<(), SchedulerError> {
        let mut task_map = self.task_map.lock().unwrap();
        
        if let Some(task) = task_map.get_mut(&task_id) {
            if task.state == TaskState::Suspended {
                task.state = TaskState::Ready;
                self.update_stats();
            }
            Ok(())
        } else {
            Err(SchedulerError::TaskNotFound(task_id))
        }
    }
    
    /// Set task deadline
    /// 
    /// # Arguments
    /// * `task_id` - The task ID
    /// * `deadline` - The deadline instant
    /// 
    /// # Returns
    /// * `Result<(), SchedulerError>` - Ok if successful, error otherwise
    pub fn set_task_deadline(&self, task_id: TaskId, deadline: Instant) -> Result<(), SchedulerError> {
        let mut task_map = self.task_map.lock().unwrap();
        
        if let Some(task) = task_map.get_mut(&task_id) {
            task.set_deadline(deadline);
            Ok(())
        } else {
            Err(SchedulerError::TaskNotFound(task_id))
        }
    }
    
    /// Get scheduler statistics
    pub fn get_stats(&self) -> SchedulerStats {
        *self.stats.lock().unwrap()
    }
    
    /// Start the scheduler
    pub fn start(&self) -> Result<(), SchedulerError> {
        let mut running = self.running.lock().unwrap();
        if *running {
            return Err(SchedulerError::SchedulerNotRunning);
        }
        *running = true;
        drop(running);
        
        let tasks = Arc::clone(&self.tasks);
        let task_map = Arc::clone(&self.task_map);
        let stats = Arc::clone(&self.stats);
        let running = Arc::clone(&self.running);
        let config = self.config;
        
        thread::spawn(move || {
            while *running.lock().unwrap() {
                let start_time = Instant::now();
                
                // Execute ready tasks
                let mut tasks_to_execute = Vec::new();
                {
                    let mut tasks = tasks.lock().unwrap();
                    let mut task_map = task_map.lock().unwrap();
                    
                    // Find ready tasks
                    let mut temp_queue = BinaryHeap::new();
                    while let Some(mut task) = tasks.pop() {
                        if task.is_ready() {
                            tasks_to_execute.push(task.id);
                        } else {
                            temp_queue.push(task);
                        }
                    }
                    *tasks = temp_queue;
                }
                
                // Execute tasks
                for task_id in tasks_to_execute {
                    let should_readd = {
                        let mut task_map = task_map.lock().unwrap();
                        if let Some(task) = task_map.get_mut(&task_id) {
                            let _result = task.execute();
                            
                            // Check for missed deadlines
                            if config.enable_deadline_monitoring && task.has_missed_deadline() {
                                let mut stats = stats.lock().unwrap();
                                stats.missed_deadlines += 1;
                            }
                            
                            // Return whether task should be re-added to queue
                            task.state == TaskState::Ready
                        } else {
                            false
                        }
                    };
                    
                    // Re-add to queue if still ready
                    if should_readd {
                        let mut tasks = tasks.lock().unwrap();
                        let task_map = task_map.lock().unwrap();
                        if let Some(task) = task_map.get(&task_id) {
                            tasks.push(task.clone());
                        }
                    }
                }
                
                // Update statistics
                {
                    let mut stats = stats.lock().unwrap();
                    stats.total_ticks += 1;
                }
                
                // Sleep for tick interval
                let elapsed = start_time.elapsed();
                let sleep_duration = Duration::from_millis(config.tick_interval_ms as u64)
                    .saturating_sub(elapsed);
                thread::sleep(sleep_duration);
            }
        });
        
        Ok(())
    }
    
    /// Stop the scheduler
    pub fn stop(&self) {
        let mut running = self.running.lock().unwrap();
        *running = false;
    }
    
    /// Check if scheduler is running
    pub fn is_running(&self) -> bool {
        *self.running.lock().unwrap()
    }
    
    /// Rebuild the task priority queue
    fn rebuild_task_queue(&self) {
        let mut tasks = self.tasks.lock().unwrap();
        let task_map = self.task_map.lock().unwrap();
        
        let mut new_queue = BinaryHeap::new();
        for task in task_map.values() {
            if task.state == TaskState::Ready {
                new_queue.push(task.clone());
            }
        }
        
        *tasks = new_queue;
    }
    
    /// Update scheduler statistics
    fn update_stats(&self) {
        let mut stats = self.stats.lock().unwrap();
        let task_map = self.task_map.lock().unwrap();
        
        stats.total_tasks = task_map.len();
        stats.ready_tasks = 0;
        stats.running_tasks = 0;
        stats.blocked_tasks = 0;
        stats.suspended_tasks = 0;
        stats.completed_tasks = 0;
        
        for task in task_map.values() {
            match task.state {
                TaskState::Ready => stats.ready_tasks += 1,
                TaskState::Running => stats.running_tasks += 1,
                TaskState::Blocked => stats.blocked_tasks += 1,
                TaskState::Suspended => stats.suspended_tasks += 1,
                TaskState::Completed => stats.completed_tasks += 1,
            }
        }
    }
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scheduler_creation() {
        let scheduler = Scheduler::new();
        assert!(!scheduler.is_running());
    }

    #[test]
    fn test_add_task() {
        let scheduler = Scheduler::new();
        let task_id = scheduler.add_task(
            "test_task".to_string(),
            TaskPriority::Normal,
            || TaskResult::Completed,
            None,
        );
        assert!(task_id.is_ok());
    }

    #[test]
    fn test_remove_task() {
        let scheduler = Scheduler::new();
        let task_id = scheduler.add_task(
            "test_task".to_string(),
            TaskPriority::Normal,
            || TaskResult::Completed,
            None,
        ).unwrap();
        
        let result = scheduler.remove_task(task_id);
        assert!(result.is_ok());
    }

    #[test]
    fn test_suspend_resume_task() {
        let scheduler = Scheduler::new();
        let task_id = scheduler.add_task(
            "test_task".to_string(),
            TaskPriority::Normal,
            || TaskResult::Completed,
            None,
        ).unwrap();
        
        assert!(scheduler.suspend_task(task_id).is_ok());
        assert!(scheduler.resume_task(task_id).is_ok());
    }
}

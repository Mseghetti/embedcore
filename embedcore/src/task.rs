//! Task management for EmbedCore

// Removed unused imports
use std::cmp::Ordering;
use std::time::{Duration, Instant};

/// Task priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TaskPriority {
    Low = 0,
    Normal = 1,
    High = 2,
    Critical = 3,
}

/// Task execution state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskState {
    Ready,
    Running,
    Blocked,
    Suspended,
    Completed,
}

/// Task function wrapper that can be cloned
#[derive(Clone)]
pub struct TaskFunctionWrapper {
    inner: std::sync::Arc<std::sync::Mutex<Box<dyn FnMut() -> TaskResult + Send + Sync>>>,
}

impl std::fmt::Debug for TaskFunctionWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TaskFunctionWrapper")
            .field("inner", &"<function>")
            .finish()
    }
}

impl TaskFunctionWrapper {
    pub fn new<F>(f: F) -> Self 
    where 
        F: FnMut() -> TaskResult + Send + Sync + 'static,
    {
        Self {
            inner: std::sync::Arc::new(std::sync::Mutex::new(Box::new(f))),
        }
    }
    
    pub fn call(&self) -> TaskResult {
        let mut f = self.inner.lock().unwrap();
        f()
    }
}

/// Task function type
pub type TaskFunction = TaskFunctionWrapper;

/// Task execution result
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskResult {
    /// Task completed successfully
    Completed,
    /// Task should be rescheduled
    Reschedule,
    /// Task should be suspended
    Suspend,
    /// Task encountered an error
    Error,
}

/// Task identifier
pub type TaskId = u32;

/// Task structure
#[derive(Clone, Debug)]
pub struct Task {
    pub id: TaskId,
    pub name: String,
    pub priority: TaskPriority,
    pub state: TaskState,
    pub function: TaskFunction,
    pub period_ms: Option<u32>,
    pub deadline: Option<Instant>,
    pub created_at: Instant,
    pub last_run: Option<Instant>,
    pub run_count: u32,
}

impl Task {
    /// Create a new task
    /// 
    /// # Arguments
    /// * `id` - Unique task identifier
    /// * `name` - Task name for debugging
    /// * `priority` - Task priority
    /// * `function` - Task function to execute
    /// * `period_ms` - Optional period for periodic tasks
    /// 
    /// # Returns
    /// * `Task` - The created task
    pub fn new(
        id: TaskId,
        name: String,
        priority: TaskPriority,
        function: TaskFunction,
        period_ms: Option<u32>,
    ) -> Self {
        Task {
            id,
            name,
            priority,
            state: TaskState::Ready,
            function,
            period_ms,
            deadline: None,
            created_at: Instant::now(),
            last_run: None,
            run_count: 0,
        }
    }
    
    /// Execute the task
    /// 
    /// # Returns
    /// * `TaskResult` - The result of task execution
    pub fn execute(&mut self) -> TaskResult {
        self.state = TaskState::Running;
        self.last_run = Some(Instant::now());
        self.run_count += 1;
        
        let result = self.function.call();
        
        match result {
            TaskResult::Completed => {
                if self.period_ms.is_some() {
                    self.state = TaskState::Ready;
                } else {
                    self.state = TaskState::Completed;
                }
            }
            TaskResult::Reschedule => {
                self.state = TaskState::Ready;
            }
            TaskResult::Suspend => {
                self.state = TaskState::Suspended;
            }
            TaskResult::Error => {
                self.state = TaskState::Suspended;
            }
        }
        
        result
    }
    
    /// Check if the task is ready to run
    pub fn is_ready(&self) -> bool {
        if self.state != TaskState::Ready {
            return false;
        }
        
        // Check if periodic task is due
        if let Some(period_ms) = self.period_ms {
            if let Some(last_run) = self.last_run {
                let elapsed = last_run.elapsed();
                return elapsed >= Duration::from_millis(period_ms as u64);
            }
        }
        
        true
    }
    
    /// Set task deadline
    pub fn set_deadline(&mut self, deadline: Instant) {
        self.deadline = Some(deadline);
    }
    
    /// Check if task has missed its deadline
    pub fn has_missed_deadline(&self) -> bool {
        if let Some(deadline) = self.deadline {
            return Instant::now() > deadline;
        }
        false
    }
}

impl PartialEq for Task {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for Task {}

impl PartialOrd for Task {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Task {
    fn cmp(&self, other: &Self) -> Ordering {
        // Higher priority tasks come first
        match other.priority.cmp(&self.priority) {
            Ordering::Equal => {
                // If priorities are equal, earlier deadline comes first
                match (self.deadline, other.deadline) {
                    (Some(self_deadline), Some(other_deadline)) => {
                        self_deadline.cmp(&other_deadline)
                    }
                    (Some(_), None) => Ordering::Less,
                    (None, Some(_)) => Ordering::Greater,
                    (None, None) => Ordering::Equal,
                }
            }
            other => other,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_task_creation() {
        let task = Task::new(
            1,
            "test_task".to_string(),
            TaskPriority::Normal,
            TaskFunction::new(|| TaskResult::Completed),
            None,
        );
        
        assert_eq!(task.id, 1);
        assert_eq!(task.name, "test_task");
        assert_eq!(task.priority, TaskPriority::Normal);
        assert_eq!(task.state, TaskState::Ready);
    }

    #[test]
    fn test_task_execution() {
        let mut task = Task::new(
            1,
            "test_task".to_string(),
            TaskPriority::Normal,
            TaskFunction::new(|| TaskResult::Completed),
            None,
        );
        
        let result = task.execute();
        assert_eq!(result, TaskResult::Completed);
        assert_eq!(task.state, TaskState::Completed);
        assert_eq!(task.run_count, 1);
    }

    #[test]
    fn test_periodic_task() {
        let mut task = Task::new(
            1,
            "periodic_task".to_string(),
            TaskPriority::Normal,
            TaskFunction::new(|| TaskResult::Completed),
            Some(100),
        );
        
        let result = task.execute();
        assert_eq!(result, TaskResult::Completed);
        assert_eq!(task.state, TaskState::Ready); // Should be ready for next period
    }

    #[test]
    fn test_task_priority_ordering() {
        let task1 = Task::new(
            1,
            "low_priority".to_string(),
            TaskPriority::Low,
            TaskFunction::new(|| TaskResult::Completed),
            None,
        );
        
        let task2 = Task::new(
            2,
            "high_priority".to_string(),
            TaskPriority::High,
            TaskFunction::new(|| TaskResult::Completed),
            None,
        );
        
        assert!(task2 > task1); // Higher priority should come first
    }
}

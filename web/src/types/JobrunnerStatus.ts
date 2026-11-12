import JobStatus from './JobStatus';

// A running job as get_jobrunner_status() in scap/spiderpig/api.py reports it.
export interface RunningJob {
	id: number
	type: string
	status: JobStatus
	data: Record<string, any>
}

// Keep in sync with get_jobrunner_status() in scap/spiderpig/api.py.
export default interface JobrunnerStatus {
	status: string
	running_jobs: RunningJob[]
	busy_queues: string[]
}

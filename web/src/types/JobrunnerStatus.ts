import Job from './Job';

// Keep in sync with jobrunner_status() in scap/spiderpig/api.py.
export default interface JobrunnerStatus {
	status: string
	running_jobs: Job[]
	busy_queues: string[]
}

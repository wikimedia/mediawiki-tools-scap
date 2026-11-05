import Interaction from './Interaction';
import JobStatus from './JobStatus';

// A job as get_job(), get_jobs() and jobrunner_status() in scap/spiderpig/api.py
// return it.  Keep in sync with prepare_job_for_response() in that file.
export default interface Job {
	id: number
	type: string
	user: string
	command: string
	command_decoded: string
	queue_name: string
	queued_at: number
	started_at: number | null
	finished_at: number | null
	exit_status: number | null
	status: JobStatus
	data: Record<string, any>
	// The pending interaction, if any.
	interaction: Interaction | null
	running: boolean
	orphaned: boolean
	// Null until the job starts.
	duration: number | null
}

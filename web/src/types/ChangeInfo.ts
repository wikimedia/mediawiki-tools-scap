import LinkSegment from './LinkSegment';

// A change of a backport job.  Keep in sync with _create_backport_job() and
// load_job_data() in scap/spiderpig/api.py.
export default interface ChangeInfo {
	linkifiedCommitMsg: Array<string | LinkSegment>
	subject: string
	project: string
	branch: string
	number: number
	url: string
	repoQueryUrl: string
	branchQueryUrl: string
}
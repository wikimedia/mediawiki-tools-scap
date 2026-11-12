import SpiderpigProgressReportRecord from "./SpiderpigProgressReportRecord"

// Keep in sync with Job.extract_status() in scap/spiderpig/model.py.
export default interface JobStatus {
	status: string|null
	progress: SpiderpigProgressReportRecord|null
}

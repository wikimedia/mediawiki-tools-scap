import SpiderpigProgressReportRecord from "./SpiderpigProgressReportRecord"

export default interface JobStatus {
	status: string|null
	progress: SpiderpigProgressReportRecord|null
}

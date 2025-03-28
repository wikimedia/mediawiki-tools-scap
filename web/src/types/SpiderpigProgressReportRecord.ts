// Keep in sync with scap/spiderpig/io.py
export default interface SpiderpigProgressReportRecord {
    name: string;
    totalTasks: number
    tasksInFlight: number|null
    tasksFinishedOk: number
    tasksFinishedFailed: number
    tasksFinishedTotal: number
    tasksPercentComplete: number
    tasksRemaining: number
    progressFinished: boolean
}

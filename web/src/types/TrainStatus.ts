import TrainGroup from "./TrainGroup"

// Keep in sync with TrainStatus in scap/spiderpig/model.py.
export default interface TrainStatus extends Record<string, any> {
	groups: Array<TrainGroup>
    version: string
    oldVersion: string | null
    atGroup: string | null
    wikiVersions: Record<string, string>
    wikiCount: number
    task: string | null
    taskUrl: string | null
    taskStatus: string | null
    taskReleaseDate: string | null
    taskVersion: string | null
    warnings: Array<string>
}

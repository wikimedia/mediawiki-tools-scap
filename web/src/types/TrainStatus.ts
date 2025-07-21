import TrainGroup from "./TrainGroup"

// Keep in sync with scap/spiderpig/model.py:TrainStatus
export default interface TrainStatus extends Record<string, any> {
	groups: Array<TrainGroup>
    version: string
    oldVersion?: string
    atGroup?: string
    wikiVersions: Record<string, string>
    wikiCount: number
    task?: string
    taskUrl?: string
    taskStatus?: string
    taskReleaseDate?: string
    taskVersion?: string
    warnings: Array<string>
}

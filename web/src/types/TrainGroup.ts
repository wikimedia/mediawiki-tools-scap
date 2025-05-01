export default interface TrainGroup extends Record<string, any> {
	name: string
	versions: Array<string>
	wikis: Array<string>
	excludedWikis: Array<string>
	selectedWikis: Array<string>
	hasRolled: boolean
	step: number
}

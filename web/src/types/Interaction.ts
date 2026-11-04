// An interaction as get_parsed_interaction() in scap/spiderpig/api.py returns it.  Keep
// in sync with that function and with the Interaction model in scap/spiderpig/model.py.
export default interface Interaction {
	id: number
	job_id: number
	type: string
	prompt: string
	// Maps the label of each choice to its code.  Empty unless the type is "choices".
	choices: Record<string, string>
	default?: string | null
	details_title?: string | null
	has_details?: boolean
	responded_by?: string | null
	response?: string | null
}

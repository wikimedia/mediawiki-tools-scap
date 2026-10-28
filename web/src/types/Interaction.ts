// Keep in sync with scap/spiderpig/model.py
export default interface Interaction {
	id: number,
	job_id: number
	type: string
	prompt: string
	choices?: string
	default?: string
	details_title?: string
	has_details?: boolean
	responded_by?: string
	response?: string
}

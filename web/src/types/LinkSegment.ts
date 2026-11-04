// One segment of a message that the server linkified. A segment that is plain
// text is a string instead.  Keep in sync with linkify_commit_mesage() in
// scap/spiderpig/api.py.
export default interface LinkSegment {
	type: 'link'
	href: string
	text: string
}

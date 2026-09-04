// One segment of a message that the server linkified. A segment that is plain
// text is a string instead.
export default interface LinkSegment {
	href: string
	text: string
}

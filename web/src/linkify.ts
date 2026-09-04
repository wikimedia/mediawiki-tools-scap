import LinkSegment from './types/LinkSegment';

/**
 * Renders the segments of a linkified message as HTML. The server escapes the
 * text of each segment before it marks up the task numbers and the change ids,
 * so the result is safe to pass to v-html.
 */
export function formatLinkifiedMessage(
	segments: Array<string | LinkSegment>,
	linkClass: string
): string {
	let res = '';
	for ( const segment of segments ) {
		if ( typeof segment === 'string' ) {
			res += segment;
		} else {
			res += `<a href="${ segment.href }" target="_blank" class="${ linkClass }">${ segment.text }</a>`;
		}
	}
	return res;
}

const relativeTime = new Intl.RelativeTimeFormat( 'en', { numeric: 'auto' } );

const ageUnits: [ Intl.RelativeTimeFormatUnit, number ][] = [
	[ 'year', 365 * 24 * 3600 ],
	[ 'month', 30 * 24 * 3600 ],
	[ 'day', 24 * 3600 ],
	[ 'hour', 3600 ],
	[ 'minute', 60 ]
];

/**
 * Formats a unix timestamp (in seconds) as its age. For example, "3 days ago".
 */
export function formatAge( timestamp: number ): string {
	const seconds = Date.now() / 1000 - timestamp;
	for ( const [ unit, unitSeconds ] of ageUnits ) {
		if ( seconds >= unitSeconds ) {
			return relativeTime.format( -Math.floor( seconds / unitSeconds ), unit );
		}
	}
	return 'just now';
}

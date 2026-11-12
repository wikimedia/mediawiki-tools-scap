import { useNow } from '@vueuse/core';

const relativeTime = new Intl.RelativeTimeFormat( 'en', { numeric: 'auto' } );

const ageUnits: [ Intl.RelativeTimeFormatUnit, number ][] = [
	[ 'year', 365 * 24 * 3600 ],
	[ 'month', 30 * 24 * 3600 ],
	[ 'day', 24 * 3600 ],
	[ 'hour', 3600 ],
	[ 'minute', 60 ]
];

/**
 * Formats a unix timestamp (in seconds) as its age at the given time. For example,
 * "3 days ago".
 */
export function formatAge( timestamp: number, now: Date = new Date() ): string {
	const seconds = now.getTime() / 1000 - timestamp;
	for ( const [ unit, unitSeconds ] of ageUnits ) {
		if ( seconds >= unitSeconds ) {
			return relativeTime.format( -Math.floor( seconds / unitSeconds ), unit );
		}
	}
	return 'just now';
}

/**
 * Returns a function like formatAge() that measures against the current time.  A
 * render or computed that calls the function is evaluated again each minute, so the
 * ages that it shows count up.
 */
export function useAgeFormatter(): ( timestamp: number ) => string {
	const minuteClock = useNow( { interval: 60000 } );
	return ( timestamp ) => {
		// Reading the clock makes the caller depend on it.  The age itself uses the
		// current time, which can be up to a minute ahead of the clock.
		void minuteClock.value;
		return formatAge( timestamp );
	};
}

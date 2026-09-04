import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { formatAge } from './time';

const NOW = 1700000000;

describe( 'formatAge', () => {
	beforeEach( () => {
		vi.useFakeTimers();
		vi.setSystemTime( NOW * 1000 );
	} );

	afterEach( () => {
		vi.useRealTimers();
	} );

	const ages: [ number, string ][] = [
		[ 0, 'just now' ],
		[ 59, 'just now' ],
		[ 60, '1 minute ago' ],
		[ 90, '1 minute ago' ],
		[ 2 * 3600, '2 hours ago' ],
		[ 24 * 3600, 'yesterday' ],
		[ 3 * 24 * 3600, '3 days ago' ],
		[ 30 * 24 * 3600, 'last month' ],
		[ 400 * 24 * 3600, 'last year' ],
		[ 800 * 24 * 3600, '2 years ago' ]
	];

	it.each( ages )( 'formats an age of %i seconds', ( seconds, expected ) => {
		expect( formatAge( NOW - seconds ) ).toBe( expected );
	} );

	it( 'formats a timestamp in the future', () => {
		expect( formatAge( NOW + 3600 ) ).toBe( 'just now' );
	} );
} );

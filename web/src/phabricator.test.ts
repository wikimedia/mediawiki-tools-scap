import { describe, expect, it } from 'vitest';
import { taskSearchUrl } from './phabricator';

const queryOf = ( message: string ): string => {
	const query = new URL( taskSearchUrl( message ) ).searchParams.get( 'query' );
	return query === null ? '' : query;
};

describe( 'taskSearchUrl', () => {
	it( 'quotes each word of the message', () => {
		expect( queryOf( 'Exception: no such file' ) )
			.toBe( '"Exception:" "no" "such" "file"' );
	} );

	it( 'searches the open tasks of the bug subtypes, newest first', () => {
		expect( taskSearchUrl( 'boom' ) ).toBe(
			'https://phabricator.wikimedia.org/maniphest/' +
			'?statuses=open()&group=none' +
			'&subtypes=bug,error,security,default,deadline&order=newest' +
			'&query=%22boom%22#R'
		);
	} );

	it( 'drops a quote of the message, which the search treats as special', () => {
		expect( queryOf( 'Call to a member function get() on "null"' ) )
			.toBe( '"Call" "to" "a" "member" "function" "get()" "on" "null"' );
	} );
} );

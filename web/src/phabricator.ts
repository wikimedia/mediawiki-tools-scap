const searchUrl = 'https://phabricator.wikimedia.org/maniphest/';

/**
 * Builds the URL of a Phabricator search for the open tasks of an error
 * message. Each word becomes a separate quoted term, because Phabricator
 * search gives a colon a special meaning. A loose match like this one finds a
 * related task, and the exact message often finds none.
 *
 * The apiserver already strips the prefix that MediaWiki puts before the
 * message of an exception. See error_message in scap/logstash.py.
 *
 * This is a port of makePhabSearchUrl in the Phatality plugin.
 */
export function taskSearchUrl( message: string ): string {
	const title = message.replace( /"+/g, ' ' ).trim();
	const query = encodeURIComponent( '"' + title.replace( /\s+/g, '" "' ) + '"' );

	return `${ searchUrl }?statuses=open()&group=none` +
		'&subtypes=bug,error,security,default,deadline&order=newest' +
		`&query=${ query }#R`;
}

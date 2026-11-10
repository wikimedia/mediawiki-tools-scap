import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { enableAutoUnmount, mount } from '@vue/test-utils';
import { nextTick } from 'vue';
import ErrorLogsCount from './ErrorLogsCount.vue';
import { errorTotal } from '../events';

vi.mock( '../api', () => ( {
	default: () => ( {
		// The tests set errorTotal instead of using the event stream.
		getEvents: () => new Promise( () => {} )
	} )
} ) );

enableAutoUnmount( afterEach );

describe( 'ErrorLogsCount', () => {
	beforeEach( () => {
		errorTotal.value = null;
	} );

	it( 'shows the error total from the event stream', async () => {
		const wrapper = mount( ErrorLogsCount );
		expect( wrapper.text() ).toBe( '⌚' );

		errorTotal.value = 12;
		await nextTick();

		expect( wrapper.text() ).toBe( '12' );
	} );
} );

import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { enableAutoUnmount, mount } from '@vue/test-utils';
import { nextTick } from 'vue';
import ErrorLogsCount from './ErrorLogsCount.vue';
import { errorTotal, staleErrorPollTime } from '../events';

vi.mock( '../api', () => ( {
	default: () => ( {
		// The tests set the event refs instead of using the event stream.
		getEvents: () => new Promise( () => {} )
	} )
} ) );

enableAutoUnmount( afterEach );

describe( 'ErrorLogsCount', () => {
	beforeEach( () => {
		vi.useFakeTimers();
		vi.setSystemTime( new Date( '2026-11-03T12:00:00Z' ) );
		errorTotal.value = null;
		staleErrorPollTime.value = null;
	} );

	afterEach( () => {
		vi.useRealTimers();
	} );

	it( 'shows the error total from the event stream', async () => {
		const wrapper = mount( ErrorLogsCount );
		const chip = wrapper.find( '.cdx-info-chip' );
		expect( chip.text() ).toBe( '⌚' );

		errorTotal.value = 12;
		await nextTick();

		expect( chip.text() ).toBe( '12' );
		expect( chip.classes() ).toContain( 'cdx-info-chip--notice' );
		expect( chip.attributes( 'title' ) ).toBeUndefined();
	} );

	it( 'warns when the last logstash poll is too old', async () => {
		errorTotal.value = 12;
		const wrapper = mount( ErrorLogsCount );
		staleErrorPollTime.value = Date.parse( '2026-11-03T11:55:00Z' ) / 1000;
		await nextTick();
		const chip = wrapper.find( '.cdx-info-chip' );

		expect( chip.text() ).toBe( '12' );
		expect( chip.classes() ).toContain( 'cdx-info-chip--warning' );
		expect( chip.attributes( 'title' ) ).toBe( 'The last successful logstash poll was 5 minutes ago' );
	} );
} );

import { describe, expect, it } from 'vitest';
import { mount } from '@vue/test-utils';
import { createMemoryHistory, createRouter } from 'vue-router';
import { createVuetify } from 'vuetify';
import { VApp } from 'vuetify/components/VApp';
import Navigation from './Navigation.vue';

// jsdom has no ResizeObserver, which the Vuetify layout needs.
global.ResizeObserver = class {
	observe() {}
	unobserve() {}
	disconnect() {}
};

const stub = { template: '<div />' };

const router = createRouter( {
	history: createMemoryHistory(),
	routes: [
		{ path: '/', component: stub },
		{ path: '/mediawiki/backport', component: stub },
		{ path: '/mediawiki/train', component: stub },
		{ path: '/mediawiki/logs', component: stub },
		{ path: '/service/deploy', component: stub },
		{ path: '/jobs/:jobId', component: stub },
		{ path: '/admin', component: stub }
	]
} );

async function mountNavigation( path: string ) {
	await router.replace( path );
	await router.isReady();

	const wrapper = mount(
		{ components: { Navigation, VApp }, template: '<v-app><navigation /></v-app>' },
		{
			global: {
				plugins: [ router, createVuetify() ],
				stubs: { UserMenu: stub, ErrorLogsCount: stub }
			}
		}
	);
	await flush();
	return wrapper;
}

async function flush() {
	await new Promise( ( resolve ) => setTimeout( resolve ) );
}

function selectedTabs( wrapper ) {
	return wrapper.findAll( '.v-tab--selected' ).map( ( tab ) => tab.attributes( 'href' ) );
}

describe( 'SpNavigation tabs', () => {
	it( 'selects the tab of the current route', async () => {
		const wrapper = await mountNavigation( '/mediawiki/train' );
		expect( selectedTabs( wrapper ) ).toEqual( [ '/mediawiki/train' ] );
	} );

	it( 'selects no tab on a route that no tab points at', async () => {
		const wrapper = await mountNavigation( '/jobs/7' );
		expect( selectedTabs( wrapper ) ).toEqual( [] );
	} );

	it( 'deselects the tab when the user leaves it', async () => {
		const wrapper = await mountNavigation( '/service/deploy' );
		expect( selectedTabs( wrapper ) ).toEqual( [ '/service/deploy' ] );

		await router.push( '/jobs/7' );
		await flush();
		expect( selectedTabs( wrapper ) ).toEqual( [] );
	} );
} );

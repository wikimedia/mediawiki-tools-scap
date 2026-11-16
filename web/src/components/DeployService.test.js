import { describe, it, expect, vi, beforeEach } from 'vitest';
import { flushPromises } from '@vue/test-utils';
import SpDeployService from './DeployService.vue';

const getServiceReleasesMock = vi.fn();
const startDeployServiceMock = vi.fn();

vi.mock( '../api', () => ( {
	default: () => ( {
		getServices: async () => ( { services: [ 'shellbox' ], errors: {} } ),
		getServiceReleases: getServiceReleasesMock,
		startDeployService: startDeployServiceMock
	} )
} ) );

vi.mock( '../jobrunner', async ( importOriginal ) => ( {
	...( await importOriginal() ),
	default: () => ( {
		queueIsBusy: () => false
	} )
} ) );

vi.mock( '../state', () => ( {
	notificationsStore: () => ( {
		setupUserNotificationsForJob: async () => {}
	} )
} ) );

describe( 'SpDeployService releases', () => {
	beforeEach( () => {
		getServiceReleasesMock.mockReset();
		startDeployServiceMock.mockReset();
	} );

	it( 'lists the releases when the drop-down opens, not before', async () => {
		getServiceReleasesMock.mockResolvedValue( {
			releases: [ 'shellbox-media/main', 'shellbox/main' ]
		} );
		const setupResult = SpDeployService.setup();

		setupResult.service.value = 'shellbox';
		await flushPromises();
		expect( getServiceReleasesMock ).not.toHaveBeenCalled();

		await setupResult.loadReleases( true );

		expect( getServiceReleasesMock ).toHaveBeenCalledWith( 'shellbox' );
		expect( setupResult.releases.value ).toEqual( [ 'shellbox-media/main', 'shellbox/main' ] );
		expect( setupResult.releasesLoading.value ).toBe( false );
	} );

	it( 'asks for the releases of a service one time', async () => {
		getServiceReleasesMock.mockResolvedValue( { releases: [ 'shellbox/main' ] } );
		const setupResult = SpDeployService.setup();

		setupResult.service.value = 'shellbox';
		await flushPromises();
		await setupResult.loadReleases( true );
		await setupResult.loadReleases( true );

		// A service that is chosen again shows what it loaded before.
		setupResult.service.value = 'echostore';
		await flushPromises();
		expect( setupResult.releases.value ).toEqual( [] );
		setupResult.service.value = 'shellbox';
		await flushPromises();
		expect( setupResult.releases.value ).toEqual( [ 'shellbox/main' ] );

		await setupResult.loadReleases( true );
		expect( getServiceReleasesMock ).toHaveBeenCalledTimes( 1 );
	} );

	it( 'asks again after a failure', async () => {
		getServiceReleasesMock.mockRejectedValueOnce( new Error( 'helmfile failed' ) );
		getServiceReleasesMock.mockResolvedValueOnce( { releases: [ 'shellbox/main' ] } );
		const setupResult = SpDeployService.setup();

		setupResult.service.value = 'shellbox';
		await flushPromises();
		await setupResult.loadReleases( true );
		expect( setupResult.noReleasesText.value ).toBe( 'helmfile failed' );

		await setupResult.loadReleases( true );
		expect( setupResult.releases.value ).toEqual( [ 'shellbox/main' ] );
		expect( getServiceReleasesMock ).toHaveBeenCalledTimes( 2 );
	} );

	it( 'deploys every release unless a release is chosen', async () => {
		getServiceReleasesMock.mockResolvedValue( { releases: [ 'shellbox/main' ] } );
		startDeployServiceMock.mockResolvedValue( { id: 7 } );
		const setupResult = SpDeployService.setup();
		setupResult.service.value = 'shellbox';
		setupResult.message.value = 'bump image';
		await flushPromises();

		await setupResult.startDeployService();
		expect( startDeployServiceMock ).toHaveBeenLastCalledWith(
			expect.objectContaining( { service: 'shellbox', release: null } )
		);

		setupResult.release.value = 'shellbox/main';
		setupResult.message.value = 'bump image';
		await setupResult.startDeployService();
		expect( startDeployServiceMock ).toHaveBeenLastCalledWith(
			expect.objectContaining( { service: 'shellbox', release: 'shellbox/main' } )
		);
	} );

	it( 'clears the release after the deployment starts', async () => {
		getServiceReleasesMock.mockResolvedValue( { releases: [ 'shellbox/main' ] } );
		startDeployServiceMock.mockResolvedValue( { id: 7 } );
		const setupResult = SpDeployService.setup();
		setupResult.service.value = 'shellbox';
		await flushPromises();
		setupResult.release.value = 'shellbox/main';
		setupResult.message.value = 'bump image';

		await setupResult.startDeployService();

		expect( setupResult.release.value ).toBe( null );
		expect( setupResult.message.value ).toBe( '' );
	} );

	it( 'drops the releases of a service that is no longer chosen', async () => {
		let answer;
		getServiceReleasesMock.mockReturnValueOnce(
			new Promise( ( resolve ) => {
				answer = resolve;
			} )
		);
		const setupResult = SpDeployService.setup();

		setupResult.service.value = 'shellbox';
		await flushPromises();
		setupResult.loadReleases( true );
		await flushPromises();
		setupResult.service.value = null;
		await flushPromises();
		answer( { releases: [ 'shellbox/main' ] } );
		await flushPromises();

		expect( setupResult.releases.value ).toEqual( [] );
		expect( setupResult.releasesLoading.value ).toBe( false );
	} );
} );

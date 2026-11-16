<template>
	<cdx-field id="deploy-service" class="deploy-service">
		<template #description>
			To run scap deploy-service, choose a service and say why you're deploying it
		</template>

		<cdx-message
			v-if="catalogErrors.length"
			class="deploy-service__catalog-errors"
			type="warning"
		>
			<p>These services have an error in the service catalog. They cannot be deployed.</p>
			<ul>
				<li v-for="[ name, error ] in catalogErrors" :key="name">
					<strong>{{ name }}</strong>: {{ error }}
				</li>
			</ul>
		</cdx-message>

		<div class="deploy-service__input">
			<div class="deploy-service__choices">
				<v-autocomplete
					v-model="service"
					:items="services"
					:loading="loading"
					aria-label="Choose a service to deploy"
					auto-select-first
					clearable
					density="compact"
					hide-details
					persistent-placeholder
					placeholder="Service"
					variant="outlined"
				>
					<template #no-data>
						<v-list-item>{{ noServicesText }}</v-list-item>
					</template>
				</v-autocomplete>

				<v-select
					v-model="release"
					:items="releases"
					:disabled="!service"
					:loading="releasesLoading"
					aria-label="Choose a release to deploy"
					:list-props="releaseListProps"
					clearable
					density="compact"
					hide-details
					persistent-placeholder
					placeholder="All releases"
					variant="outlined"
					@update:menu="loadReleases"
				>
					<template #no-data>
						<v-list-item>{{ noReleasesText }}</v-list-item>
					</template>
				</v-select>
			</div>

			<v-text-field
				v-model="message"
				aria-label="Log message that says why"
				density="compact"
				hide-details
				persistent-placeholder
				placeholder="Log message that says why"
				variant="outlined"
				@keydown.enter.stop.prevent="startDeployService"
			>
				<template #append>
					<cdx-button
						:disabled="buttonDisabled"
						:title="buttonDisabledReason"
						@click="startDeployService"
					>
						Deploy Service
					</cdx-button>
				</template>
			</v-text-field>
		</div>

		<cdx-checkbox v-model="confirmDiffs">
			Show the diffs and ask for approval before the deployment
		</cdx-checkbox>

		<cdx-dialog
			:open="alertDialogOpen"
			title="Failed to start deployment"
			@update:open="alertDialogOpen = $event"
		>
			{{ alertDialogText }}
		</cdx-dialog>
	</cdx-field>
</template>

<script lang="ts">
import { ref, computed, onMounted, watch } from 'vue';
import { CdxButton, CdxCheckbox, CdxDialog, CdxField, CdxMessage } from '@wikimedia/codex';
import { VAutocomplete } from 'vuetify/components/VAutocomplete';
import { VListItem } from 'vuetify/components/VList';
import { VSelect } from 'vuetify/components/VSelect';
import { VTextField } from 'vuetify/components/VTextField';
import useApi, { type ApiError } from '../api';
import useJobrunner, { serviceQueue } from '../jobrunner';
import { notificationsStore } from '../state';

export default {
	name: 'SpDeployService',
	components: {
		CdxButton,
		CdxCheckbox,
		CdxDialog,
		CdxField,
		CdxMessage,
		VAutocomplete,
		VListItem,
		VSelect,
		VTextField
	},

	setup() {
		const api = useApi();
		const jobrunner = useJobrunner();
		const notifications = notificationsStore();

		const services = ref<string[]>( [] );
		const service = ref<string | null>( null );
		const releases = ref<string[]>( [] );
		// "namespace/release", or null for every release.
		const release = ref<string | null>( null );
		const releasesLoading = ref( false );
		const releasesLoadError = ref<string | null>( null );
		// The releases of each service that the drop-down has loaded.
		const loadedReleases = new Map<string, string[]>();
		// VSelect names its list `${label}-list`.  This select carries an
		// aria-label instead of a label, so the list would be "undefined-list".
		// The type of `list-props` does not cover aria-label, which VSelect
		// nonetheless passes on to the list.
		const releaseListProps = { 'aria-label': 'Releases' } as any;

		const message = ref( '' );
		const confirmDiffs = ref( true );
		const loading = ref( false );
		const loadError = ref<string | null>( null );
		// The [ name, error ] pair of each service entry that is not valid.
		const catalogErrors = ref<[string, string][]>( [] );
		const alertDialogOpen = ref( false );
		const alertDialogText = ref( '' );

		const noServicesText = computed(
			() => loadError.value || 'No services found.'
		);

		const noReleasesText = computed( () => {
			if ( releasesLoading.value ) {
				return 'Loading the releases';
			}
			return releasesLoadError.value || 'No releases found.';
		} );

		const serviceIsDeploying = computed(
			() => !!service.value && jobrunner.queueIsBusy( serviceQueue( service.value ) )
		);

		const buttonDisabledReason = computed( () => {
			if ( !service.value ) {
				return 'Choose a service';
			}
			if ( !message.value.trim() ) {
				return 'Enter a log message that says why';
			}
			if ( serviceIsDeploying.value ) {
				return `${ service.value } is deploying`;
			}
			return '';
		} );

		const buttonDisabled = computed( () => buttonDisabledReason.value !== '' );

		async function loadServices() {
			loading.value = true;
			try {
				const res = await api.getServices();
				services.value = res.services;
				catalogErrors.value = Object.entries( res.errors || {} );
				loadError.value = null;
			} catch ( error ) {
				const apiError = error as ApiError;
				loadError.value = apiError.respJson?.detail?.message || apiError.message;
			}
			loading.value = false;
		}

		function showReleases( name: string | null ) {
			release.value = null;
			releasesLoading.value = false;
			releasesLoadError.value = null;
			releases.value = ( name && loadedReleases.get( name ) ) || [];
		}

		// A request runs `helmfile list` once per namespace and environment of
		// the service, which takes a second or more.  The releases load when the
		// drop-down opens, and one time per service.
		async function loadReleases( open: boolean ) {
			const name = service.value;
			if ( !open || !name || loadedReleases.has( name ) || releasesLoading.value ) {
				return;
			}

			releasesLoading.value = true;
			let found: string[] = [];
			let failure: string | null = null;
			try {
				found = ( await api.getServiceReleases( name ) ).releases;
			} catch ( error ) {
				const apiError = error as ApiError;
				failure = apiError.respJson?.detail?.message || apiError.message;
			}

			// Drop the answer for a service that is no longer chosen.
			if ( service.value !== name ) {
				return;
			}

			if ( failure === null ) {
				loadedReleases.set( name, found );
			}
			releasesLoading.value = false;
			releasesLoadError.value = failure;
			releases.value = found;
		}

		async function startDeployService() {
			if ( buttonDisabled.value ) {
				return;
			}

			try {
				const res = await api.startDeployService( {
					service: service.value,
					message: message.value.trim(),
					confirmDiffs: confirmDiffs.value,
					release: release.value
				} );
				await notifications.setupUserNotificationsForJob( res.id );
				message.value = '';
				release.value = null;
			} catch ( error ) {
				alertDialogOpen.value = true;
				const apiError = error as ApiError;
				alertDialogText.value = apiError.respJson?.detail?.message || apiError.message;
			}
		}

		onMounted( loadServices );
		watch( service, showReleases );

		return {
			alertDialogOpen,
			alertDialogText,
			buttonDisabled,
			buttonDisabledReason,
			catalogErrors,
			confirmDiffs,
			loading,
			loadReleases,
			message,
			noReleasesText,
			noServicesText,
			release,
			releaseListProps,
			releases,
			releasesLoading,
			service,
			services,
			startDeployService
		};
	}
};
</script>

<style lang="less">
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

.deploy-service {
	&.cdx-field {
		margin-bottom: @spacing-150;
	}

	&__catalog-errors {
		margin-bottom: @spacing-50;
	}

	&__input {
		display: flex;
		flex-direction: column;
		gap: @spacing-50;

		.v-input__append {
			margin-inline-start: @spacing-50;

			.cdx-button {
				min-height: 40px;
			}
		}
	}

	&__choices {
		display: flex;
		gap: @spacing-50;
	}

	.cdx-checkbox {
		margin-top: @spacing-50;
	}
}
</style>

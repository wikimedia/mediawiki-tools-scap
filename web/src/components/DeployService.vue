<template>
	<cdx-field id="deploy-service" class="deploy-service">
		<template #description>
			To run scap deploy-service, choose a service and say why you're deploying it
		</template>

		<div class="deploy-service__input">
			<v-autocomplete
				v-model="service"
				:disabled="!idle"
				:items="services"
				:loading="loading"
				aria-label="Choose a service to deploy"
				auto-select-first
				clearable
				density="compact"
				persistent-placeholder
				placeholder="Service"
				variant="outlined"
			>
				<template #no-data>
					<v-list-item>{{ noServicesText }}</v-list-item>
				</template>
			</v-autocomplete>

			<v-text-field
				v-model="message"
				:disabled="!idle"
				aria-label="Log message that says why"
				density="compact"
				persistent-placeholder
				placeholder="Log message that says why"
				variant="outlined"
				@keydown.enter.stop.prevent="startDeployService"
			>
				<template #append>
					<cdx-button
						:disabled="buttonDisabled"
						@click="startDeployService"
					>
						Deploy Service
					</cdx-button>
				</template>
			</v-text-field>
		</div>

		<cdx-checkbox v-model="confirmDiffs" :disabled="!idle">
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
import { ref, computed, onMounted } from 'vue';
import { CdxButton, CdxCheckbox, CdxDialog, CdxField } from '@wikimedia/codex';
import { VAutocomplete } from 'vuetify/components/VAutocomplete';
import { VListItem } from 'vuetify/components/VList';
import { VTextField } from 'vuetify/components/VTextField';
import useApi from '../api';
import { notificationsStore } from '../state';

export default {
	name: 'SpDeployService',
	components: {
		CdxButton,
		CdxCheckbox,
		CdxDialog,
		CdxField,
		VAutocomplete,
		VListItem,
		VTextField
	},

	props: {
		idle: {
			type: Boolean
		}
	},

	setup( props ) {
		const api = useApi();
		const notifications = notificationsStore();

		const services = ref<string[]>( [] );
		const service = ref<string | null>( null );
		const message = ref( '' );
		const confirmDiffs = ref( false );
		const loading = ref( false );
		const loadError = ref<string | null>( null );
		const alertDialogOpen = ref( false );
		const alertDialogText = ref( '' );

		const noServicesText = computed(
			() => loadError.value || 'No services found.'
		);

		const buttonDisabled = computed(
			() => !props.idle || !service.value || !message.value.trim()
		);

		async function loadServices() {
			loading.value = true;
			try {
				const res = await api.getServices();
				services.value = res.services;
				loadError.value = null;
			} catch ( error ) {
				loadError.value = error.respJson?.detail?.message || error.message;
			}
			loading.value = false;
		}

		async function startDeployService() {
			if ( buttonDisabled.value ) {
				return;
			}

			try {
				const res = await api.startDeployService( {
					service: service.value,
					message: message.value.trim(),
					confirmDiffs: confirmDiffs.value
				} );
				await notifications.setupUserNotificationsForJob( res.id );
				message.value = '';
			} catch ( error ) {
				alertDialogOpen.value = true;
				alertDialogText.value = error.respJson?.detail?.message || error.message;
			}
		}

		onMounted( loadServices );

		return {
			alertDialogOpen,
			alertDialogText,
			buttonDisabled,
			confirmDiffs,
			loading,
			message,
			noServicesText,
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

	&__input {
		display: flex;
		gap: @spacing-50;

		.v-input__append {
			margin-inline-start: 0;

			.cdx-button {
				border-top-left-radius: 0px;
				border-bottom-left-radius: 0px;
				min-height: 40px;
			}
		}
	}
}
</style>

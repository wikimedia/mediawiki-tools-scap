<template>
	<div class="job-viewer-page">
		<v-toolbar v-if="job">
			<template #title>
				<h2>Job #{{ job.id }}</h2>
			</template>
			<template #append>
				<v-btn
					variant="elevated"
					@click="close()"
				>
					Close
				</v-btn>
				<v-menu>
					<template #activator="{ props }">
						<v-btn
							v-show="job.running"
							variant="elevated"
							v-bind="props"
						>
							<cdx-icon :icon="cdxIconEllipsis" />
						</v-btn>
					</template>
					<v-list>
						<v-list-item
							v-for="( item, i ) in menuItems"
							:key="i"
							v-bind="item"
							@click="onSelect( item.value )"
						/>
					</v-list>
				</v-menu>
			</template>
		</v-toolbar>
		<v-sheet color="surface-light">
			<sp-interaction
				v-if="interaction"
				:interaction="interaction"
			/>
			<sp-job-card
				v-if="job"
				:key="job.id"
				v-bind="job"
			/>
			<sp-interaction
				id="bottom-interaction"
				v-if="interaction"
				:interaction="interaction"
			/>
		</v-sheet>
	</div>
</template>

<script lang="ts">
import { defineComponent, ref, watch } from 'vue';
import { useRouter } from 'vue-router';
import { VSheet } from 'vuetify/components/VSheet';
import { VToolbar } from 'vuetify/components/VToolbar';
import useApi from '../api';
import { useJobUpdates } from '../events';
import Interaction from '../types/Interaction';
import Job from '../types/Job';
import SpInteraction from './Interaction.vue';
import SpJobCard from './JobCard.vue';
import { CdxIcon } from '@wikimedia/codex';
import { cdxIconEllipsis } from '@wikimedia/codex-icons';

export default defineComponent( {
	name: 'JobViewerPage',

	components: {
		SpInteraction,
		SpJobCard,
		CdxIcon,
		VSheet,
		VToolbar
	},

	props: {
		jobId: {
			type: String,
			required: true,
			default: null
		}
	},

	setup( props ) {
		// Pinia store.
		const api = useApi();
		const router = useRouter();

		const close = () => {
			if ( window.history.state.back ) {
				router.back();
			} else {
				router.push( '/' );
			}
		};

		// Non-reactive data.
		const menuItems = [
			{ title: 'Interrupt Job', value: 'interrupt' },
			{ title: 'Kill Job (not recommended)', value: 'kill', class: 'bg-error' }
		];

		// Reactive data properties.
		const job = ref<Job | null>( null );
		const interaction = ref<Interaction | null>( null );

		// Methods.
		const loadJob = async () => {
			const jobId = props.jobId;
			const jobInfo = await api.getJobInfo( jobId );
			if ( jobId !== props.jobId ) {
				return;
			}
			job.value = jobInfo.job;
			interaction.value = jobInfo.job.interaction;
		};

		const clickSignalButton = ( action: string ) => {
			api.signalJob( props.jobId, action );
		};

		const onSelect = ( newSelection: string ) => {
			// Handle menu button events.
			clickSignalButton( newSelection );
		};

		// Lifecycle hooks.
		watch( () => props.jobId, () => {
			job.value = null;
			interaction.value = null;
		} );

		useJobUpdates( () => props.jobId, loadJob );

		return {
			job,
			interaction,
			cdxIconEllipsis,
			menuItems,
			onSelect,
			close
		};
	}
} );
</script>

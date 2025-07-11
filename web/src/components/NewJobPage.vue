<template>
	<v-toolbar v-if="title">
		<template #title>
			<h2>{{ title }}</h2>
		</template>
	</v-toolbar>
	<v-sheet color="surface-light">
		<slot name="new-job" :idle="idle" />
	</v-sheet>
	<sp-job-history />
	<sp-logs />
</template>

<script lang="ts">
import { defineComponent } from 'vue';
import { VSheet } from 'vuetify/components/VSheet';
import { VToolbar } from 'vuetify/components/VToolbar';
import useJobrunner from '../jobrunner';
import SpJobHistory from './JobHistory.vue';
import SpLogs from './Log.vue';

export default defineComponent( {
	name: 'SpNewJobPage',
	components: {
		SpJobHistory,
		SpLogs,
		VSheet,
		VToolbar
	},

	props: {
		title: {
			type: String,
			default: ''
		}
	},

	setup() {
		const jobrunner = useJobrunner();

		return {
			idle: jobrunner.idle
		};
	}
} );
</script>

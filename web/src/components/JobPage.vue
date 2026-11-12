<template>
	<div class="job-page">
		<v-toolbar v-if="title">
			<template #title>
				<h2>{{ title }}</h2>
			</template>
		</v-toolbar>
		<v-sheet color="surface-light">
			<slot name="new-job" />
		</v-sheet>
		<sp-job-history :job-type="jobType" />
	</div>
</template>

<script lang="ts">
import { defineComponent } from 'vue';
import { VSheet } from 'vuetify/components/VSheet';
import { VToolbar } from 'vuetify/components/VToolbar';
import SpJobHistory from './JobHistory.vue';

export default defineComponent( {
	name: 'SpJobPage',
	components: {
		SpJobHistory,
		VSheet,
		VToolbar
	},

	props: {
		title: {
			type: String,
			default: ''
		},
		// The kind of job that this page starts.  The job history shows only
		// jobs of this kind, unless the user asks for all of them.
		jobType: {
			type: String,
			default: null
		}
	}
} );
</script>

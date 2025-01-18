<template>
	<div id="job-history" class="job-history">
		<h2 class="job-history__heading">
			Job History
		</h2>

		<!-- Column labels -->
		<div class="job-history__column-labels">
			<div class="job-history__column-labels__label">
				Id
			</div>

			<div class="job-history__column-labels__label">
				Command
			</div>

			<div class="job-history__column-labels__label">
				User
			</div>

			<div class="job-history__column-labels__label">
				Queued
			</div>

			<div class="job-history__column-labels__label">
				Started
			</div>

			<div class="job-history__column-labels__label">
				Finished
			</div>

			<div class="job-history__column-labels__label">
				Status
			</div>
		</div>

		<!-- Job cards -->
		<div class="job-history__card-list">
			<div v-if="jobs.length > 0">
				<sp-job-card
					v-for="job in jobs"
					:key="job.id"
					v-bind="job"
					class="job-history__card-list__card"
				/>
			</div>
			<!-- Empty and loading state -->
			<div v-else>
				<p v-if="loaded">
					No job data to display.
				</p>
				<p v-else>
					Loading...
				</p>
			</div>
		</div>
	</div>
</template>

<script lang="ts">
import { ref, onMounted, onUnmounted } from 'vue';
import SpJobCard from './JobCard.vue';
import useApi from '../api';

const INTERVAL = 1000;
const JOB_RUNNING = '..Running...';

export default {
	name: 'SpJobHistory',
	components: {
		SpJobCard
	},
	emits: [
		'rowClicked'
	],
	setup() {
		// Pinia store.
		const api = useApi();

		// Reactive data properties.
		const loaded = ref( false );
		const jobs = ref( [] );
		const intervalTimer = ref( null );

		async function loadHistory() {
			try {
				const apiResponse = await api.getJobs( 5, 0 );
				const apiJobs = apiResponse.jobs;

				for ( const job of apiJobs ) {
					// eslint-disable-next-line camelcase
					job.command_decoded = JSON.parse( job.command ).join( ' ' );
					// eslint-disable-next-line camelcase
					job.finished_at_message = job.finished_at;

					if ( job.started_at && !job.finished_at ) {
						// eslint-disable-next-line camelcase
						job.finished_at_message = JOB_RUNNING;
					}
				}

				jobs.value = apiJobs;
				loaded.value = true;
			} catch ( error ) {
				// eslint-disable-next-line no-console
				console.error( error.message );
			}
		}

		onMounted( () => {
			loadHistory();
			intervalTimer.value = setInterval( loadHistory, INTERVAL );
		} );

		onUnmounted( () => {
			if ( intervalTimer.value ) {
				clearInterval( intervalTimer.value );
				intervalTimer.value = null;
			}
		} );

		return {
			jobs,
			loaded
		};
	}
};
</script>

<style lang="less">
@import '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';
@import '../mixins/job-grid.less';

.job-history {
	&__heading {
		font-size: @font-size-x-large;
		margin-top: @spacing-100;
		margin-bottom: @spacing-100;
		padding-bottom: @spacing-25;
		border-bottom: @border-subtle;
	}

	&__column-labels {
		// Align column labels with CdxCard padding.
		padding: 0 12px;
		// Hide and show column labels based on media queries.
		display: none;

		// Apply grid style and column labels on mid to large screen devices.
		@media screen and ( min-width: @min-width-breakpoint-tablet ) {
			.sp-mixin-job-grid();
		}

		&__label {
			font-weight: @font-weight-bold;
		}
	}

	&__card-list__card {
		margin-bottom: @spacing-50;

			&:last-child {
				margin-bottom: 0;
			}
	}
}
</style>

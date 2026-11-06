<template>
	<v-sheet id="job-history" class="job-history">
		<div class="job-history__heading">
			<h2>{{ heading }}</h2>
			<cdx-checkbox
				v-if="jobType"
				v-model="showAllTypes"
				:inline="true"
			>
				Show all job types
			</cdx-checkbox>
		</div>

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
				Started
			</div>

			<div class="job-history__column-labels__label">
				Finished
			</div>

			<div class="job-history__column-labels__label">
				Status
			</div>

			<div class="job-history__column-labels__label">
				Actions
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
				<cdx-button v-if="possiblyMoreHistory" @click="loadMoreHistory">
					Load more history
				</cdx-button>
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
	</v-sheet>
</template>

<script lang="ts">
import { ref, computed, watch } from 'vue';
import { VSheet } from 'vuetify/components/VSheet';
import { CdxButton, CdxCheckbox } from '@wikimedia/codex';
import SpJobCard from './JobCard.vue';
import useApi from '../api';
import { useJobListUpdates } from '../events';
import Job from '../types/Job';

export default {
	name: 'SpJobHistory',
	components: {
		SpJobCard,
		CdxButton,
		CdxCheckbox,
		VSheet
	},
	props: {
		// A JobType value from scap/spiderpig/model.py, or null to show
		// every kind of job.
		jobType: {
			type: String,
			default: null
		}
	},
	emits: [
		'rowClicked'
	],
	setup( props ) {
		// Pinia store.
		const api = useApi();

		// Reactive data properties.
		const loaded = ref( false );
		const jobs = ref<Job[]>( [] );
		const possiblyMoreHistory = ref( false );
		const showAllTypes = ref( false );

		const selectedType = computed(
			() => ( showAllTypes.value ? null : props.jobType )
		);

		const heading = computed( () => ( ( {
			backport: 'Backport History',
			train: 'Train History',
			'deploy-service': 'Service Deployment History'
		} as Record<string, string> )[ selectedType.value ?? '' ] ?? 'Job History' ) );

		let numJobsToDisplay = 5;

		async function loadHistory() {
			try {
				const apiResponse = await api.getJobs(
					numJobsToDisplay, 0, selectedType.value
				);
				const apiJobs = apiResponse.jobs;

				jobs.value = apiJobs;
				loaded.value = true;
				possiblyMoreHistory.value = ( apiJobs.length === numJobsToDisplay );
			} catch ( error ) {
				console.error( ( error as Error ).message );
			}
		}

		const update = useJobListUpdates( loadHistory );

		function loadMoreHistory() {
			numJobsToDisplay += 5;
			update();
		}

		watch( selectedType, () => {
			numJobsToDisplay = 5;
			jobs.value = [];
			loaded.value = false;
			update();
		} );

		return {
			heading,
			jobs,
			loaded,
			loadMoreHistory,
			possiblyMoreHistory,
			showAllTypes
		};
	}
};
</script>

<style lang="less">
@import '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';
@import '../mixins/job-grid.less';

.job-history {
	&__heading {
		display: flex;
		align-items: baseline;
		justify-content: space-between;
		gap: @spacing-100;
		margin-top: @spacing-100;
		margin-bottom: @spacing-100;
		padding-bottom: @spacing-25;
		border-bottom: @border-subtle;

		h2 {
			font-size: @font-size-x-large;
		}
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

<template>
	<v-row>
		<v-col :cols="12" :md="6" :class="mdAndUp ? '' : 'pb-0'">
			<v-card
				prepend-icon="mdi-calendar"
				rounded
				class="mb-2">
				<template #title>
					Week {{ currentWeek }} of MediaWiki Train
				</template>
				<template v-if="hasLoaded && !error" #subtitle>
					Task <a :href="taskURL" target="_blank">{{ task }}</a>
				</template>
				<template v-else #subtitle>
					Fetching current train status...
					<v-progress-circular size="small" color="primary" indeterminate />
				</template>
			</v-card>
		</v-col>
		<v-col :cols="12" :md="6" :class="mdAndUp ? '' : 'pt-0'">
			<v-card
				v-if="hasLoaded && !error"
				prepend-icon="mdi-code-braces-box"
				rounded
				class="mb-2">
				<template #title>
					Deployment of <v-chip color="primary" size="small">
						{{ trainVersion }}
					</v-chip>
				</template>
				<template #subtitle>
					Currently at {{ trainIsAt }}
				</template>
			</v-card>
			<v-alert v-if="error" type="error" closable>
				{{ error }}
			</v-alert>
			<v-alert v-for="warning in warnings" :key="warning" type="warning" closable>
				{{ warning }}
			</v-alert>
			<v-alert v-if="isBackporting" type="warning">
				Backports are in progress. Train deployment is unavailable until they complete.
			</v-alert>
		</v-col>
	</v-row>

	<v-stepper
		v-if="hasLoaded && !error"
		v-model="selectedStep"
		:items="groupNames"
		:disabled="isRolling"
		alt-labels
		editable
		hide-actions
	>
		<template #icon="{ step }">
			<template v-if="step === selectedStep">
				<v-icon icon="mdi-train" />
			</template>
			<template v-else-if="step === 1">
				<v-icon icon="mdi-train-car-caboose" />
			</template>
			<template v-else-if="selectedStep < step">
				<v-icon v-if="groups[step - 1].hasRolled" icon="mdi-history" />
				<v-icon v-else icon="mdi-calendar-clock" />
			</template>
			<template v-else>
				<v-icon v-if="groups[step - 1].hasRolled" icon="mdi-check-bold" />
				<v-icon v-else icon="mdi-progress-alert" />
			</template>
		</template>

		<template v-for="group of groups" :key="group.name" #['header-item.'+group.step]>
			<template v-if="deployment.rollback && group.hasRolled && selectedStep < group.step">
				<v-chip
					:class="{ processing: isRolling }"
					variant="flat"
					color="secondary"
					size="small">
					{{ formatVersion( deployment.version ) }}
				</v-chip>
				←
			</template>
			<template v-if="mdAndUp || ( group.hasRolled && !( selectedStep < group.step ) )">
				<template v-for="version in group.versions" :key="version">
					<v-chip
						:color="version === trainVersion ? 'primary' : 'secondary'"
						size="small"
					>
						{{ formatVersion( version ) }}
					</v-chip>
				</template>
			</template>
			<template v-if="deployment.promote && !group.hasRolled && selectedStep >= group.step">
				→
				<v-chip
					:class="{ processing: isRolling }"
					variant="flat"
					color="primary"
					size="small">
					{{ formatVersion( deployment.version ) }}
				</v-chip>
			</template>
		</template>

		<template v-for="group of groups" :key="group.name" #['item.'+group.step]>
			<v-autocomplete
				v-model="group.excludedWikis"
				:label="'Exclude ' + group.name + ' wikis'"
				:items="group.wikis"
				item-color="warning"
				multiple
				clearable
				hide-details
				:disabled="!deployment.promote"
			>
				<template #chip="{ props }">
					<v-chip
						v-bind="props"
						color="warning"
						size="large" />
				</template>
			</v-autocomplete>
		</template>

		<v-card prepend-icon="mdi-train" title="Deployment Summary">
			<template v-if="deployment.noop" #title>
				Currently at {{ deployment.group.name }}
			</template>
			<template v-else #title>
				<template v-if="deployment.promote">
					<span v-if="isRolling">Promoting</span>
					<span v-else>Promote</span>
				</template>
				<template v-else>
					<span v-if="isRolling">Rolling back</span>
					<span v-else>Roll back</span>
				</template>
				{{ deployment.version }}
				from {{ trainIsAt }} to {{ deployment.group.name }}

				<v-progress-circular v-if="isRolling" size="small" color="primary" indeterminate />
			</template>

			<template #subtitle>
				Number of wikis on each version following deployment:
			</template>

			<template #text>
				<v-alert
					v-if="deployment.excludedWikis.length"
					type="warning"
					class="mb-4"
				>
					<template #text>
						Excluding {{ deployment.excludedWikis.length }} wikis from promotion.
					</template>
				</v-alert>
				<v-progress-linear
					v-for="version in deployment.versions"
					:key="version"
					:model-value="versionPercent( deployment.versionCounts[version] )"
					:buffer-value="versionPercent( priorVersionCounts[version] )"
					:color="version === trainVersion ? 'primary' : 'secondary'"
					height="42"
				>
					<v-chip
						:color="version === trainVersion ? 'primary' : 'secondary'"
						size="small"
						variant="flat"
					>
						<strong>{{ version }}</strong>
						on
						<strong>{{ deployment.versionCounts[version] }}</strong> wikis
					</v-chip>
				</v-progress-linear>
			</template>

			<template #actions>
				<v-btn
					v-if="deployment.rollback"
					color="warning"
					variant="flat"
					:disabled="isDisabled"
					@click="startTrain"
				>
					Roll back!
				</v-btn>
				<v-btn
					v-else
					color="primary"
					variant="flat"
					:disabled="isDisabled"
					@click="startTrain"
				>
					Let's roll!
				</v-btn>
			</template>
		</v-card>
	</v-stepper>
</template>

<script lang="ts">
import { computed, ref, onMounted, watch } from 'vue';
import { useDisplay } from 'vuetify';
import { VAlert } from 'vuetify/components/VAlert';
import { VAutocomplete } from 'vuetify/components/VAutocomplete';
import { VBtn } from 'vuetify/components/VBtn';
import { VCard } from 'vuetify/components/VCard';
import { VChip } from 'vuetify/components/VChip';
import { VCol, VRow } from 'vuetify/components/VGrid';
import { VIcon } from 'vuetify/components/VIcon';
import { VProgressLinear } from 'vuetify/components/VProgressLinear';
import { VStepper } from 'vuetify/components/VStepper';
import useApi from '../api';
import useJobrunner from '../jobrunner';
import TrainGroup from '../types/TrainGroup';

export default {
	name: 'SpTrain',
	components: {
		VAlert,
		VAutocomplete,
		VBtn,
		VCard,
		VChip,
		VCol,
		VIcon,
		VProgressLinear,
		VRow,
		VStepper
	},
	setup() {
		const { mdAndUp } = useDisplay();
		const jobrunner = useJobrunner();

		let resetWikiVersions = {};
		let originalTrainStatus = {};

		// Reactive properties
		const hasLoaded = ref( false );
		const error = ref( null );
		const warnings = ref( [] );
		const jobPending = ref( false );

		const groups = ref<TrainGroup[]>( [] );
		const selectedStep = ref( 1 );
		const trainIsAt = ref( '' );
		const trainVersion = ref( '' );
		const oldVersion = ref( '' );
		const task = ref( '' );
		const taskURL = ref( '' );
		const priorVersionCounts = ref( {} );
		const totalWikis = ref( 0 );

		// Computed properties
		const groupNames = computed( () => groups.value.map( ( group ) => group.name ) );

		const currentWeek = computed( () => {
			const now = new Date();
			const ny = new Date( now.getFullYear(), 0, 1 );

			return Math.ceil( ( now.getTime() - ny.getTime() ) / ( 1000 * 60 * 60 * 24 ) / 7 );
		} );

		const deployment = computed( () => {
			const from = groupNames.value.indexOf( trainIsAt.value );
			const to = selectedStep.value - 1;
			const wikiVersions = { ...resetWikiVersions };
			const excludedWikis = [];
			const version = from > to ? oldVersion.value : trainVersion.value;

			for ( let i = 0; i < selectedStep.value; i++ ) {
				const group = groups.value[ i ];
				const excluded = new Set( group.excludedWikis );

				for ( const wiki of group.wikis ) {
					if ( excluded.has( wiki ) ) {
						excludedWikis.push( wiki );
					} else {
						wikiVersions[ wiki ] = trainVersion.value;
					}
				}
			}

			const versionCounts = countVersions(
				wikiVersions,
				oldVersion.value,
				trainVersion.value
			);

			return {
				noop: from === to,
				promote: from < to,
				rollback: from > to,
				group: groups.value[ to ],
				wikiVersions: wikiVersions,
				version: version,
				oldVersion: oldVersion.value,
				versionCounts: versionCounts,
				versions: Object.keys( versionCounts ).sort().reverse(),
				excludedWikis: from <= to ? excludedWikis : []
			};
		} );

		const isRolling = computed(
			() => jobrunner.status.value?.job?.type === 'train' || jobPending.value
		);
		const isBackporting = computed( () => jobrunner.status.value?.job?.type === 'backport' );
		const isDisabled = computed(
			() => deployment.value.noop || isRolling.value || isBackporting.value
		);

		/**
		 * Starts a train promotion job using the currently computed deployment that is based on the
		 * selected step. If the "start" step is selected, a complete rollback is performed.
		 */
		async function startTrain() {
			const api = useApi();
			const deploy = deployment.value;
			let promotion;

			if ( deploy.rollback && deploy.group.name === 'start' ) {
				// Complete rollback. "Promote" all to old version.
				promotion = {
					group: 'all',
					version: oldVersion.value,
					oldVersion: oldVersion.value
				};
			} else {
				promotion = {
					group: deploy.group.name,
					version: trainVersion.value,
					oldVersion: oldVersion.value,
					excludedWikis: deploy.excludedWikis
				};
			}

			// Memoize the original train status along with changes to the selected wikis so that we
			// can accurately represent progress during deployments to all SpiderPig users.
			for ( let i = 1; i < groups.value.length; i++ ) {
				originalTrainStatus.groups[ i - 1 ].selectedWikis = groups.value[ i ].selectedWikis;
			}

			promotion.originalTrainStatus = originalTrainStatus;

			jobPending.value = true;
			await api.startTrain( promotion );
		}

		/**
		 * Returns an integer percentage value for the given number of wikis relative to the total
		 * number of wikis.
		 *
		 * @param {number} numWikis Number of wikis.
		 *
		 * @return {number}
		 */
		function versionPercent( numWikis ) {
			return Math.round( numWikis / totalWikis.value * 100 );
		}

		/**
		 * Formats the MW train version number. Only the "wmf.x" suffix is displayed to fit within
		 * the stepper item flex basis.
		 *
		 * @param {string} version MW version.
		 *
		 * @return {string}
		 */
		function formatVersion( version ) {
			return version.slice( version.lastIndexOf( '-' ) + 1 );
		}

		/**
		 * Returns a mapping of version numbers to the number of wikis on each version.
		 *
		 * @param {Object} wikiversions Wiki to version mapping.
		 * @param {...string} knownVersions Other versions to include in the count.
		 *
		 * @return {Object} Mapping of version to count of wikis on each version.
		 */
		function countVersions( wikiversions, ...knownVersions ) {
			return Object.values( wikiversions ).reduce(
				( counts, version ) => (
					{ ...counts, [ version ]: ( counts[ version ] || 0 ) + 1 }
				),
				knownVersions.reduce( ( counts, version ) => (
					{ ...counts, [ version ]: 0 } ), {}
				)
			);
		}

		/**
		 * Retrieves train status from the API and updates refs. If the job runner is currently
		 * processing a train job, the job's memoized train status is preferred.
		 */
		async function refresh() {
			const api = useApi();

			try {
				const [ trainStatus, jrStatus ] = await Promise.all( [
					api.trainStatus(),
					api.getJobrunnerStatus()
				] );

				originalTrainStatus = trainStatus;

				if ( jrStatus?.job?.type === 'train' && jrStatus?.job?.data?.originalTrainStatus ) {
					updateFromTrainStatus(
						jrStatus.job.data.originalTrainStatus,
						jrStatus.job.data?.group
					);
				} else {
					updateFromTrainStatus( trainStatus, null );
				}
			} catch ( e ) {
				error.value = e.message;
			} finally {
				hasLoaded.value = true;
			}
		}

		// Update responsive state from the given train status.
		function updateFromTrainStatus( train, selectedGroup ) {
			groups.value = [
				{
					name: 'start',
					versions: [],
					wikis: [],
					selectedWikis: [],
					excludedWikis: [],
					hasRolled: true,
					step: 1
				}
			].concat( train.groups.map( ( group, i ) => ( {
				...group,
				step: i + 2
			} as TrainGroup ) ) );

			trainIsAt.value = train.atGroup || 'start';
			trainVersion.value = train.version;
			oldVersion.value = train.oldVersion;
			task.value = train.task;
			taskURL.value = train.taskUrl;
			resetWikiVersions = Object.keys( train.wikiVersions ).reduce(
				( vers, wiki ) => ( { [ wiki ]: train.oldVersion, ...vers } ),
				{}
			);
			priorVersionCounts.value = countVersions(
				train.wikiVersions,
				train.oldVersion,
				train.version
			);
			totalWikis.value = train.wikiCount;
			warnings.value = train.warnings;

			const grp = groups.value.find(
				( g ) => g.name === ( selectedGroup || trainIsAt.value )
			);
			selectedStep.value = grp?.step || 1;
		}

		// Refresh from current train status after jobs complete
		watch( jobrunner.idle, async ( idle ) => {
			if ( idle ) {
				await refresh();
			}
		} );

		// Whenever there is a train job being processed, favor its memoized train status over the
		// one returned by the API since the latter is based on the unsynced on-disk state. This
		// will ensure that all users see the same status during deployments.
		let prevJobID;
		watch( jobrunner.status, async ( status ) => {
			// Reset the job pending flag upon the first train or backport job status update
			if ( status?.job?.type === 'train' || status?.job?.type === 'backport' ) {
				jobPending.value = false;
			}

			if ( status?.job?.type === 'train' && status?.job?.data?.originalTrainStatus ) {

				if ( prevJobID !== status.job.id ) {
					updateFromTrainStatus(
						status?.job?.data?.originalTrainStatus,
						status?.job?.data?.group
					);
					prevJobID = status.job.id;
				}
			}
		} );

		onMounted( refresh );

		return {
			error,
			currentWeek,
			deployment,
			formatVersion,
			groupNames,
			groups,
			hasLoaded,
			isBackporting,
			isDisabled,
			isRolling,
			mdAndUp,
			priorVersionCounts,
			selectedStep,
			startTrain,
			task,
			taskURL,
			trainIsAt,
			trainVersion,
			versionPercent,
			warnings
		};
	}
};
</script>

<style lang="less" scoped>
.v-stepper {
	@media (min-width: 960px) {
		:deep(.v-stepper-item) {
			flex-basis: 200px;
		}

		:deep(.v-divider) {
			margin-left: -75px;
			margin-right: -75px;
		}
	}

	:deep(.v-stepper-item__title) {
		margin-bottom: 0.5rem;
	}

	:deep(.v-stepper-item__title) {
		margin-bottom: 0.5rem;
	}

	:deep(.v-stepper-item__avatar) {
		width: 2rem !important;
		height: 2rem !important;
	}

	:deep(.v-stepper-item__avatar .v-icon) {
		font-size: 1.5rem !important;
	}
}

.processing {
	animation: pulse 1s infinite;
}

@keyframes pulse {
	from { opacity: 1; }
	50% { opacity: 0.2; }
	to { opacity: 1; }
}
</style>

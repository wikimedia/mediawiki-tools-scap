<template>
	<v-row>
		<v-col>
			<v-card
				prepend-icon="mdi-calendar"
				rounded
				class="mb-2">
				<template #title>
					Week {{ currentWeek }} of MediaWiki Train
				</template>
				<template #subtitle>
					Deployment of <v-chip color="primary">
						{{ trainVersion }}
					</v-chip>,
					task <a :href="taskURL" target="_blank">{{ task }}</a>
				</template>
			</v-card>
		</v-col>
	</v-row>

	<v-stepper
		v-if="hasLoaded"
		v-model="selectedStep"
		:items="groupNames"
		:disabled="isProcessing"
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

		<!-- eslint-disable-next-line vue/no-parsing-error -->
		<template v-for="group of groups" :key="group.name" #['header-item.'+group.step]>
			<template v-if="deployment.rollback && group.hasRolled && selectedStep < group.step">
				<v-chip
					:class="{ processing: isProcessing }"
					color="secondary"
					size="small">
					{{ deployment.version }}
				</v-chip>
				←
			</template>
			<template v-for="version in group.versions" :key="version">
				<v-chip
					:color="version === trainVersion ? 'primary' : 'secondary'"
					size="small"
				>
					{{ version }}
				</v-chip>
			</template>
			<template v-if="deployment.promote && !group.hasRolled && selectedStep >= group.step">
				→
				<v-chip
					:class="{ processing: isProcessing }"
					color="primary"
					size="small">
					{{ deployment.version }}
				</v-chip>
			</template>
		</template>

		<!-- eslint-disable-next-line vue/no-parsing-error -->
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
			<template v-if="deployment.promote" #title>
				Promote {{ trainIsAt }} → {{ deployment.group.name }}
			</template>
			<template v-else-if="deployment.rollback" #title>
				Roll back {{ trainIsAt }} → {{ deployment.group.name }}
			</template>
			<template v-else #title>
				Currently at {{ deployment.group.name }}
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
		const jobrunner = useJobrunner();

		let resetWikiVersions = {};

		// Reactive properties
		const hasLoaded = ref( false );
		const error = ref( null );

		const groups = ref( Array<TrainGroup> );
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

			return Math.ceil( ( now - ny ) / ( 1000 * 60 * 60 * 24 ) / 7 );
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

		const isProcessing = computed( () => jobrunner.status.value?.job?.type === 'train' );

		const isDisabled = computed( () => deployment.value.noop || isProcessing.value );

		/**
		 * Starts a train promotion job using the currently computed deployment that is based on the
		 * selected step. If the "start" step is selected, a complete rollback is performed.
		 */
		async function startTrain() {
			const api = useApi();
			const deploy = deployment.value;
			let promotion;

			/* eslint-disable camelcase */
			if ( deploy.rollback && deploy.group.name === 'start' ) {
				// Complete rollback. "Promote" all to old version.
				promotion = {
					group: 'all',
					version: oldVersion.value,
					old_version: oldVersion.value
				};
			} else {
				promotion = {
					group: deploy.group.name,
					version: trainVersion.value,
					old_version: oldVersion.value,
					excluded_wikis: deploy.excludedWikis
				};
			}
			/* eslint-enable camelcase */

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
		 * Retrieves train status using the API and updates refs.
		 */
		async function refresh() {
			const api = useApi();

			try {
				const res = await api.trainStatus();

				groups.value = [
					{
						name: 'start',
						versions: [],
						wikis: [],
						selectedWikis: [],
						hasRolled: true,
						step: 1
					}
				].concat( res.groups.map( ( group, i ) => {
					const versions = res.group_versions[ group ];

					return {
						name: group,
						versions: versions,
						wikis: [ ...res.group_wikis[ group ] ],
						selectedWikis: [ ...res.group_wikis[ group ] ],
						hasRolled: versions.length === 1 && versions[ 0 ] === res.train_version,
						step: i + 2
					} as TrainGroup;
				} ) );

				trainIsAt.value = res.train_is_at || 'start';
				trainVersion.value = res.train_version;
				oldVersion.value = res.old_version;
				task.value = res.task;
				taskURL.value = res.task_url;
				resetWikiVersions = Object.keys( res.wiki_versions ).reduce(
					( vers, wiki ) => ( { [ wiki ]: res.old_version, ...vers } ),
					{}
				);
				priorVersionCounts.value = countVersions(
					res.wiki_versions,
					res.old_version,
					res.train_version
				);
				totalWikis.value = res.total_wikis;

				const currentGroup = groups.value.find( ( g ) => g.name === trainIsAt.value );
				selectedStep.value = currentGroup?.step || 1;
			} catch ( e ) {
				error.value = e.message;
			} finally {
				hasLoaded.value = true;
			}
		}

		// Refresh from current train status after jobs complete
		watch( jobrunner.idle, async ( idle ) => {
			if ( idle ) {
				await refresh();
			}
		} );

		onMounted( refresh );

		return {
			currentWeek,
			deployment,
			groupNames,
			groups,
			hasLoaded,
			isDisabled,
			isProcessing,
			priorVersionCounts,
			selectedStep,
			startTrain,
			task,
			taskURL,
			trainIsAt,
			trainVersion,
			versionPercent
		};
	}
};
</script>

<style lang="less" scoped>
.v-stepper {
	:deep(.v-stepper-item) {
		flex-basis: 300px;
	}

	:deep(.v-divider) {
		margin-left: -120px;
		margin-right: -120px;
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
	animation: pulse 2s infinite;
}

@keyframes pulse {
	from { opacity: 1; }
	50% { opacity: 0.5; }
	to { opacity: 1; }
}
</style>

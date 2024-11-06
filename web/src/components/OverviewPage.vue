<template>
	<div id="jobrunner-status">
		{{ jobrunnerStatus }}
	</div>
	<sp-backport v-if="idle" />
	<sp-interaction
		v-if="interaction"
		:interaction="interaction"
	/>
	<sp-job-history />
</template>

<script>
import useApi from '../api';
import SpBackport from './Backport.vue';
import SpJobHistory from './JobHistory.vue';
import SpInteraction from './Interaction.vue';

export default {
	name: 'OverviewPage',
	components: {
		SpBackport,
		SpJobHistory,
		SpInteraction
	},

	data() {
		return {
			api: null,
			idle: false,
			interaction: null,
			intervalTimer: null,
			jobrunnerStatus: 'Jobrunner Status: Unknown'
		};
	},

	methods: {
		async updateJobrunnerStatus() {
			try {
				const res = await this.api.getJobrunnerStatus();
				this.jobrunnerStatus = `Jobrunner Status: ${ res.status }`;
				this.idle = res.status === 'idle';
				this.interaction = res.pending_interaction;
			} catch ( error ) {
				if ( !this.api.isAuthenticated ) {
					this.$router.push( '/login' );
				} else {
					// eslint-disable-next-line no-console
					console.error( `updateJobrunnerStatus caught: ${ error.message }` );
				}
			}
		}
	},

	mounted() {
		this.api = useApi();
		this.intervalTimer = setInterval( this.updateJobrunnerStatus, 1000 );
		this.updateJobrunnerStatus();
	},

	unmounted() {
		if ( this.intervalTimer ) {
			clearInterval( this.intervalTimer );
			this.intervalTimer = null;
		}
	}
};
</script>

<style scoped>
</style>

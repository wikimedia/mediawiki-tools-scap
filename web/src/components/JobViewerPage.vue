<template>
	<div>
		<div v-if="job">
			<p>Queued by: {{ job.user }} at {{ job.queued_at }}</p>
			<p>Command: {{ job.command_decoded }}</p>
			<p v-if="summary">
				{{ summary }}
			</p>
			<p>Status: {{ job.status }}</p>
		</div>

		<sp-interaction
			v-if="interaction"
			:interaction="interaction"
		/>

		<cdx-button @click="$router.push( '/' )">
			Close Log
		</cdx-button>

		<cdx-checkbox
			v-model="showSensitive"
			type="checkbox"
			@change="onShowSensitiveChange">
			Show sensitive information
		</cdx-checkbox>

		<div v-if="jobRunning" id="signal-buttons">
			<cdx-button id="interrupt" @click="clickSignalButton">
				Interrupt Job
			</cdx-button>

			<cdx-button id="kill" @click="clickSignalButton">
				Kill Job (not recommended)
			</cdx-button>
		</div>

		<div id="terminal" ref="terminalRef" />
	</div>
</template>

<script lang="ts">
import { defineComponent, ref, onMounted, onUnmounted } from 'vue';
import '@xterm/xterm/css/xterm.css';
import { Terminal } from '@xterm/xterm';
import { FitAddon } from '@xterm/addon-fit';
import useApi from '../api';
import SpInteraction from './Interaction.vue';
import { CdxButton, CdxCheckbox } from '@wikimedia/codex';

const fitAddon = new FitAddon();

export default defineComponent( {
	name: 'JobViewerPage',

	components: {
		CdxButton,
		CdxCheckbox,
		SpInteraction
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

		// Template ref.
		const terminalRef = ref<HTMLDivElement>();

		// Reactive data properties.
		const job = ref( null );
		const term = ref( null );
		const jobRunning = ref( false );
		const monitorInterval = ref( null );
		const summary = ref( null );
		const interaction = ref( null );
		const showSensitive = ref( false );
		const abortPopulateTerminal = ref( null );

		// Methods.
		const populateTerminal = async () => {
			while ( true ) {
				term.value.clear();

				try {
					abortPopulateTerminal.value = new AbortController();

					const response = await api.getJobLog(
						props.jobId,
						abortPopulateTerminal.value.signal,
						showSensitive.value
					);
					try {
						for await ( const chunk of response.body ) {
							term.value.write( chunk );
						}
						// The complete job log has been read.
						return;
					} catch ( err ) {
						if ( err.name === 'AbortError' ) {
							if ( abortPopulateTerminal.value.signal.reason === 'showSensitive changed' ) {
								// Restart the loop if the showSensitive changed.
								continue;
							} else {
								return;
							}
						}

						// eslint-disable-next-line no-console
						console.log( `populateTerminal caught ${ err }` );
						return;
					}
				} finally {
					abortPopulateTerminal.value = null;
				}
			}
		};

		const resizeTerminal = () => {
			fitAddon.fit();
		};

		const stopMonitor = () => {
			if ( monitorInterval.value ) {
				clearInterval( monitorInterval.value );
				monitorInterval.value = null;
			}
		};

		const generateSummary = ( jobFetched ) => {
			if ( !jobFetched.started_at ) {
				return 'Job has not started yet';
			}

			let summaryText = `Job started at ${ jobFetched.started_at }`;
			if ( jobFetched.finished_at ) {
				summaryText += ` and finished at ${ jobFetched.finished_at } with exit status ${ jobFetched.exit_status }`;
			} else {
				summaryText += ' and is still running';
			}
			return summaryText;
		};

		const monitorJob = async () => {
			const jobInfo = await api.getJobInfo( props.jobId );
			const fetchedJob = jobInfo.job;

			// eslint-disable-next-line camelcase
			fetchedJob.command_decoded = JSON.parse( fetchedJob.command ).join( ' ' );
			job.value = fetchedJob;
			summary.value = generateSummary( fetchedJob );
			jobRunning.value = ( fetchedJob.started_at && !fetchedJob.finished_at );
			interaction.value = jobInfo.pending_interaction;

			if ( !jobRunning.value ) {
				stopMonitor();
			}
		};

		const clickSignalButton = ( event ) => {
			api.signalJob( props.jobId, event.target.id );
		};

		const onShowSensitiveChange = () => {
			if ( abortPopulateTerminal.value ) {
				// populateTerminal will restart itself when it receives this signal.
				abortPopulateTerminal.value.abort( 'showSensitive changed' );
			} else {
				// Restart populateTerminal if it was not running.
				populateTerminal();
			}
		};

		// Lifecycle hooks.
		onMounted( () => {
			const terminal = new Terminal( {
				convertEol: true
			} );

			terminal.loadAddon( fitAddon );

			terminal.open( terminalRef.value );
			fitAddon.fit();

			window.addEventListener( 'resize', resizeTerminal );
			term.value = terminal;
			populateTerminal();

			monitorInterval.value = setInterval( monitorJob, 1000 );
			monitorJob();
		} );

		onUnmounted( () => {
			if ( abortPopulateTerminal.value ) {
				abortPopulateTerminal.value.abort( 'unmounted' );
			}

			stopMonitor();
			window.removeEventListener( 'resize', resizeTerminal );
		} );

		return {
			job,
			summary,
			interaction,
			showSensitive,
			onShowSensitiveChange,
			jobRunning,
			clickSignalButton,
			terminalRef
		};
	}
} );
</script>

<style scoped>
#terminal {
	margin: 5px;
}
</style>

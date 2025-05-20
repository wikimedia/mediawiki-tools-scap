<template>
	<div class="job-log">
		<div class="job-log__title">
			Log
		</div>
		<v-tooltip :text="showSensitiveToolTipText" location="bottom">
			<template #activator="{ props }">
				<div class="job-log__checkbox">
					<cdx-checkbox
						v-bind="props"
						v-model="showSensitive"
						type="checkbox"
						:inline="true"
						@change="onShowSensitiveChange"
					>
						Show sensitive information
					</cdx-checkbox>
				</div>
			</template>
		</v-tooltip>
		<div id="terminal" ref="terminalRef" />
	</div>
</template>

<script lang="ts">
/* eslint-disable no-console */
import { defineComponent, onMounted, onUnmounted, ref } from 'vue';
import { VTooltip } from 'vuetify/components/VTooltip';
import { CdxCheckbox } from '@wikimedia/codex';
import useApi from '../api';
import '@xterm/xterm/css/xterm.css';
import { Terminal } from '@xterm/xterm';
import { FitAddon } from '@xterm/addon-fit';

const fitAddon = new FitAddon();

export default defineComponent( {
	name: 'SpJobLog',

	components: {
		CdxCheckbox,
		VTooltip
	},

	props: {
		/* Job id number. */
		jobId: {
			type: Number,
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
		const term = ref( null );
		const abortPopulateTerminal = ref( null );
		const showSensitive = ref( false );

		async function* logRecordsProcessor( reader ) {
			const decoder = new TextDecoder();
			let buffer = '';

			while ( true ) {
				let done: boolean, value: Uint8Array;
				try {
					( { done, value } = await reader.read() );
				} catch ( error ) {
					console.error( 'Error reading from log stream:', error );
					break;
				}

				if ( done ) {
					// Process any remaining data in the buffer
					if ( buffer ) {
						try {
							const json = JSON.parse( buffer );
							// Process the last JSON object
							yield json;
						} catch ( error ) {
							console.error( 'Error parsing final JSON:', error );
						}
					}
					break;
				}

				buffer += decoder.decode( value, { stream: true } );
				let newlineIndex: number;
				while ( ( newlineIndex = buffer.indexOf( '\n' ) ) !== -1 ) {
					const jsonString = buffer.slice( 0, newlineIndex );
					buffer = buffer.slice( newlineIndex + 1 );
					try {
						const json = JSON.parse( jsonString );
						// Process the parsed JSON object
						yield json;
					} catch ( error ) {
						console.error( 'Error parsing JSON:', error );
					}
				}
			}
		}

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
						const reader = response.body.getReader();
						let sawEOF = false;

						for await ( const rec of logRecordsProcessor( reader ) ) {
							if ( rec.type === 'line' ) {
								term.value.write( rec.line + '\n' );
							} else if ( rec.type === 'response' ) {
								term.value.write( `[User '${ rec.user }' responded with '${ rec.response }']\n` );
							} else if ( rec.type === 'EOF' ) {
								sawEOF = true;
								break;
							}
						}
						if ( sawEOF ) {
							term.value.write( '[End of job log]\n' );
						} else {
							// Assume that the connection was terminated prematurely (perhaps
							// due to timeout).  Restart the loop to recover.
							console.log( 'Job log stream terminated prematurely.  Restarting' );
							continue;
						}

						// The complete job log has been read.
						return;
					} catch ( err ) {
						if ( abortPopulateTerminal.value.signal.aborted ) {
							const reason = abortPopulateTerminal.value.signal.reason;
							if ( reason === 'showSensitive changed' ) {
								// Restart the loop if the showSensitive changed.
								continue;
							}
							if ( reason === 'unmounted' ) {
								return;
							}
							console.error( `populateTerminal: Unexpected abort reason: ${ reason }` );
							return;
						}

						console.error( 'populateTerminal unhandled error:', err );
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

		const onShowSensitiveChange = () => {
			if ( abortPopulateTerminal.value ) {
				// populateTerminal will restart itself when it receives this signal.
				abortPopulateTerminal.value.abort( 'showSensitive changed' );
			} else {
				// Restart populateTerminal if it was not running.
				populateTerminal();
			}
		};

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
		} );

		onUnmounted( () => {
			if ( abortPopulateTerminal.value ) {
				abortPopulateTerminal.value.abort( 'unmounted' );
			}

			window.removeEventListener( 'resize', resizeTerminal );
		} );

		const showSensitiveToolTipText = 'Show information from the job log that we usually want to avoid revealing' +
		' (such as security patch information or secrets) when copying and pasting the log into a Phabricator ticket.';

		return {
			terminalRef,
			showSensitive,
			onShowSensitiveChange,
			showSensitiveToolTipText
		};
	}
} );
</script>

<style lang="less">
@import '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

.job-log {
	#terminal {
		margin-top: @spacing-50;
	}

	&__title {
		margin-top: @spacing-50;
		margin-bottom: @spacing-25;
		font-size: @font-size-medium;
	}

	&__checkbox {
		text-align: end;
	}
}
</style>

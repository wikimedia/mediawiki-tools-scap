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
		<pre ref="terminalRef" class="job-log__terminal" v-html="logContent" />
		<div ref="bottomAnchorRef" id="log" class="job-log__bottom-anchor" />
	</div>
</template>

<script lang="ts">
import { defineComponent, nextTick, onMounted, onUnmounted, ref, watch } from 'vue';
import { useRoute } from 'vue-router';
import { VTooltip } from 'vuetify/components/VTooltip';
import { CdxCheckbox } from '@wikimedia/codex';
import useApi from '../api';
import { AnsiUp } from 'ansi_up';

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
		const ansiUp = new AnsiUp();
		// Don't generate links for URLs in the job log
		ansiUp.url_allowlist = {};
		// Pinia store.
		const api = useApi();
		const route = useRoute();
		const terminalRef = ref<HTMLElement|null>( null );
		const bottomAnchorRef = ref<HTMLElement|null>( null );
		// Reactive data properties.
		const logContent = ref( '' );
		const abortPopulateTerminal = ref<AbortController | null>( null );
		const showSensitive = ref( false );

		const scrollToLogBottom = async () => {
			if ( route.hash !== '#log' ) {
				return;
			}

			// Ensure that all components are mounted.
			await nextTick();
			
			const interaction = document.getElementById( 'bottom-interaction' );
			if ( interaction ) {
				interaction.scrollIntoView( { block: 'start' } );
			} else {
				bottomAnchorRef.value?.scrollIntoView( { block: 'end' } );
			}
		};

		const isTerminalScrolledToBottom = () => {
			if ( !terminalRef.value ) {
				return true;
			}

			const distanceFromBottom = terminalRef.value.scrollHeight - terminalRef.value.scrollTop - terminalRef.value.clientHeight;
			return distanceFromBottom <= 2;
		};

		const autoScrollTerminal = async () => {
			await nextTick();
			if ( terminalRef.value ) {
				terminalRef.value.scrollTop = terminalRef.value.scrollHeight;
			}
		};

		const appendLogLine = async ( line: string ) => {
			const shouldAutoScroll = isTerminalScrolledToBottom();
			logContent.value += ansiUp.ansi_to_html( line ) + '\n';
			if ( shouldAutoScroll ) {
				await autoScrollTerminal();
			}
			await scrollToLogBottom();
		};

		async function* logRecordsProcessor( reader: ReadableStreamDefaultReader<Uint8Array> ) {
			const decoder = new TextDecoder();
			let buffer = '';

			while ( true ) {
				let done: boolean, value: Uint8Array | undefined;
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
				logContent.value = '';
				await nextTick();
				if ( terminalRef.value ) {
					terminalRef.value.scrollTop = 0;
				}

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
								await appendLogLine( rec.line );
							} else if ( rec.type === 'response' ) {
								await appendLogLine( `[User '${ rec.user }' responded with '${ rec.response }']` );
							} else if ( rec.type === 'EOF' ) {
								sawEOF = true;
								break;
							}
						}
						if ( sawEOF ) {
							await appendLogLine( '[End of job log]' );
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
			populateTerminal();
			scrollToLogBottom();
		} );

		watch( () => route.hash, () => {
			scrollToLogBottom();
		} );

		onUnmounted( () => {
			if ( abortPopulateTerminal.value ) {
				abortPopulateTerminal.value.abort( 'unmounted' );
			}
		} );

		const showSensitiveToolTipText = 'Show information from the job log that we usually want to avoid revealing' +
		' (such as security patch information or secrets) when copying and pasting the log into a Phabricator ticket.';

		return {
			bottomAnchorRef,
			terminalRef,
			logContent,
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
	&__terminal {
		margin-top: @spacing-50;
		overflow-y: auto;
		max-height: 600px;
		background-color: #000;
		color: #f0f0f0;
		padding: @spacing-50;
		font-family: monospace;
		white-space: pre-wrap;
		word-break: break-all;
	}

	&__title {
		margin-top: @spacing-50;
		margin-bottom: @spacing-25;
		font-size: @font-size-medium;
	}

	&__checkbox {
		text-align: end;
	}

	&__bottom-anchor {
		height: 1px;
	}
}
</style>

<template>
	<div class="job-log">
		<div class="job-log__title">
			Log
		</div>
		<div class="job-log__action">
			<div class="job-log__action__buttons">
				<cdx-button
					class="job-log__action__buttons__close-button"
					@click="$router.push( '/' )"
				>
					Close Log
				</cdx-button>
				<cdx-menu-button
					v-show="isJobInProgress"
					v-model:selected="selection"
					:menu-items="menuItems"
					aria-label="Choose an option"
					@update:selected="onSelect"
				>
					<cdx-icon :icon="cdxIconEllipsis" />
				</cdx-menu-button>
			</div>
			<div class="job-log__action__checkbox">
				<cdx-checkbox
					v-model="showSensitive"
					type="checkbox"
					:inline="true"
					@change="onShowSensitiveChange"
				>
					Show sensitive information
				</cdx-checkbox>
			</div>
		</div>
		<div id="terminal" ref="terminalRef" />
	</div>
</template>

<script lang="ts">
import { defineComponent, onMounted, onUnmounted, ref } from 'vue';
import { CdxButton, CdxCheckbox, CdxMenuButton, CdxIcon } from '@wikimedia/codex';
import { cdxIconEllipsis } from '@wikimedia/codex-icons';
import useApi from '../api';
import '@xterm/xterm/css/xterm.css';
import { Terminal } from '@xterm/xterm';
import { FitAddon } from '@xterm/addon-fit';

const fitAddon = new FitAddon();

export default defineComponent( {
	name: 'SpJobLog',

	components: {
		CdxButton,
		CdxCheckbox,
		CdxMenuButton,
		CdxIcon
	},

	props: {
		/* Job id number. */
		jobId: {
			type: Number,
			required: true,
			default: null
		},
		/* Whether the job is in progress. */
		isJobInProgress: {
			type: Boolean,
			required: true,
			default: false
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
		const selection = ref( null );

		// Non-reactive data.
		const menuItems = [
			{ label: 'Interrupt Job', value: 'interrupt' },
			{ label: 'Kill Job (not recommended)', value: 'kill', action: 'destructive' }
		];

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

		const onShowSensitiveChange = () => {
			if ( abortPopulateTerminal.value ) {
				// populateTerminal will restart itself when it receives this signal.
				abortPopulateTerminal.value.abort( 'showSensitive changed' );
			} else {
				// Restart populateTerminal if it was not running.
				populateTerminal();
			}
		};

		const clickSignalButton = ( action ) => {
			api.signalJob( props.jobId, action );
		};

		const onSelect = ( newSelection ) => {
			// Handle menu button events.
			clickSignalButton( newSelection );

			// Reset the selection of menu buttons.
			selection.value = null;
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

		return {
			terminalRef,
			showSensitive,
			onShowSensitiveChange,
			cdxIconEllipsis,
			selection,
			menuItems,
			onSelect
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
	}

	&__action {
			display: flex;
			flex-direction: column;
			align-items: flex-end;

		&__buttons {
			display: flex;
			margin-bottom: @spacing-25;

			// Override margin.
			&__close-button.cdx-button {
				margin-right: @spacing-25;
			}
		}
	}
}
</style>

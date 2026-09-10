<template>
	<v-card flat>
		<template #text>
			<div class="error-logs__filters">
				<v-text-field
					v-model="search"
					prepend-inner-icon="mdi-magnify"
					hide-details
					single-line
					label="Filter"
				/>
				<cdx-checkbox v-model="hideNoted">
					Hide messages with notes
				</cdx-checkbox>
			</div>
		</template>

		<v-data-table
			:headers="headers"
			:items="visibleItems"
			:search="search"
			:filter-keys="[ 'versions', 'message.message', 'noteText' ]"
			item-value="message"
		>
			<template #item.message="{ value }">
				<div class="error-message">
					<a
						:href="value.link"
						class="cdx-link"
						target="_blank"
					>
						{{ value.message }}
						<cdx-icon :icon="cdxIconLinkExternal" />
					</a>
					<a
						:href="value.searchUrl"
						class="error-message__search"
						target="_blank"
						title="Search Phabricator for the open tasks with this message"
					>
						<cdx-icon :icon="cdxIconSearch" />
					</a>
				</div>
			</template>

			<template #item.note="{ item }">
				<div class="error-note">
					<!-- eslint-disable vue/no-v-html -->
					<span
						v-if="item.note"
						:title="item.note.byline"
						v-html="item.note.html"
					/>
					<!-- eslint-enable vue/no-v-html -->
					<cdx-button
						class="error-note__action"
						weight="quiet"
						:aria-label="item.note ? 'Edit the note' : 'Add a note'"
						@click="openNoteEditor( item )"
					>
						<cdx-icon
							:icon="item.note ? cdxIconEdit : cdxIconAdd"
							size="small"
						/>
					</cdx-button>
					<cdx-button
						v-if="item.note"
						class="error-note__action"
						action="destructive"
						weight="quiet"
						aria-label="Remove the note"
						@click="openNoteRemover( item )"
					>
						<cdx-icon :icon="cdxIconTrash" size="small" />
					</cdx-button>
				</div>
			</template>
		</v-data-table>

		<cdx-dialog
			:open="dialogOpen"
			:title="dialogTitle"
			close-button-label="Cancel"
			class="error-note__dialog"
			@update:open="dialogOpen = $event"
		>
			<p v-if="removing">
				Remove this note?
			</p>
			<template v-if="noteTarget">
				<p v-if="removing" class="error-note__dialog__quote">
					{{ noteTarget.text }}
				</p>
				<p class="error-note__dialog__subject">
					{{ noteTarget.subject }}
				</p>
				<p v-if="noteTarget.byline" class="error-note__dialog__byline">
					{{ noteTarget.byline }}
				</p>
			</template>
			<cdx-text-area
				v-if="!removing"
				v-model="noteDraft"
				placeholder="Enter note text"
				rows="3"
			/>
			<cdx-message v-if="noteError" type="error" inline>
				{{ noteError }}
			</cdx-message>
			<template #footer>
				<cdx-button
					:action="removing ? 'destructive' : 'progressive'"
					weight="primary"
					:disabled="confirmDisabled"
					@click="confirmDialog"
				>
					{{ confirmLabel }}
				</cdx-button>
				<cdx-button @click="dialogOpen = false">
					Cancel
				</cdx-button>
			</template>
		</cdx-dialog>
	</v-card>
</template>

<script>
import { computed, onMounted, onUnmounted, defineComponent, ref } from 'vue';
import {
	CdxButton,
	CdxCheckbox,
	CdxDialog,
	CdxIcon,
	CdxMessage,
	CdxTextArea
} from '@wikimedia/codex';
import {
	cdxIconAdd,
	cdxIconEdit,
	cdxIconLinkExternal,
	cdxIconSearch,
	cdxIconTrash
} from '@wikimedia/codex-icons';
import rison from 'rison-node';
const search = ref( '' );

import useApi from '../api';
import { formatLinkifiedMessage } from '../linkify';
import { taskSearchUrl } from '../phabricator';
import { formatAge } from '../time';

export default defineComponent( {
	name: 'SpLogs',
	components: {
		CdxButton,
		CdxCheckbox,
		CdxDialog,
		CdxIcon,
		CdxMessage,
		CdxTextArea
	},
	setup() {
		const customMessageSort = ( a, b ) => a.message.localeCompare( b.message );

		const headers = [
			{ title: 'Count', align: 'start', key: 'count' },
			{ title: 'Versions', align: 'start', key: 'versions' },
			{ title: 'Message', align: 'start', key: 'message', sort: customMessageSort },
			{ title: 'Note', align: 'start', key: 'note', sortable: false }
		];

		const items = ref();
		const hideNoted = ref( false );
		const api = useApi();

		const visibleItems = computed( () => {
			if ( !hideNoted.value ) {
				return items.value;
			}
			return items.value?.filter( ( item ) => !item.note );
		} );

		const maxMessageLength = 1000;
		const INTERVAL = 15000;
		let intervalTimer = null;

		// The row that the dialog acts on.
		const noteTarget = ref( null );
		const noteDraft = ref( '' );
		const noteError = ref( '' );
		const noteBusy = ref( false );
		const dialogOpen = ref( false );
		// 'add', 'edit' or 'remove'
		const dialogMode = ref( 'add' );
		const removing = computed( () => dialogMode.value === 'remove' );

		const dialogTitle = computed( () => {
			const titles = {
				add: 'Add note',
				edit: 'Edit note',
				remove: 'Remove note'
			};
			return titles[ dialogMode.value ];
		} );

		const confirmLabel = computed( () => {
			if ( noteBusy.value ) {
				return removing.value ? 'Removing...' : 'Saving...';
			}
			return removing.value ? 'Remove' : 'Save';
		} );

		const confirmDisabled = computed(
			() => noteBusy.value || ( !removing.value && !noteDraft.value.trim() )
		);

		const createOpenSearchLink = ( errorMessage, fieldName = 'normalized_message', timeRange = '24h' ) => {
			const dashboard = 'https://logstash.wikimedia.org/app/dashboards#/view/mediawiki-errors';
			const filter = {
				meta: {
					alias: null,
					disabled: false,
					key: fieldName,
					negate: false,
					params: { query: errorMessage },
					type: 'phrase'
				},
				query: {
					match_phrase: {
						[ fieldName ]: errorMessage
					}
				}
			};

			const globalState = {
				filters: [],
				refreshInterval: { pause: true, value: 0 },
				time: { from: `now-${ timeRange }`, to: 'now' }
			};

			const appState = {
				columns: [ '_source' ],
				filters: [ filter ],
				index: 'logstash-*',
				interval: 'auto',
				query: { language: 'kuery', query: '' },
				sort: []
			};

			return `${ dashboard }?_g=${ encodeURIComponent( rison.encode( globalState ) ) }` +
				`&_a=${ encodeURIComponent( rison.encode( appState ) ) }`;
		};

		const populateLogs = async () => {
			const resp = await api.getLogs(),
				newData = [];
			for ( const [ key, value ] of Object.entries( resp.log ) ) {
				let message = key;
				const link = createOpenSearchLink( key );
				if ( key.length > maxMessageLength ) {
					message = key.slice( 0, maxMessageLength ) + '...';
				}
				newData.push( {
					count: value.count,
					versions: value.versions.join( ' ' ),
					message: {
						message: message,
						link: link,
						searchUrl: taskSearchUrl( key )
					},
					fullMessage: key,
					noteText: value.note ? value.note.text : '',
					note: value.note ? {
						html: formatLinkifiedMessage( value.note.linkified, 'cdx-link' ),
						byline: `Set by ${ value.note.user }, ${ formatAge( value.note.updatedAt ) }`
					} : null
				} );
			}

			newData.sort( ( a, b ) => {
				if ( a.count < b.count ) {
					return 1;
				} else if ( a.count > b.count ) {
					return -1;
				}
				return 0;
			} );
			items.value = newData;
		};

		const setNoteTarget = ( item ) => {
			noteTarget.value = {
				message: item.fullMessage,
				subject: item.message.message,
				text: item.noteText,
				byline: item.note ? item.note.byline : ''
			};
			noteError.value = '';
		};

		const openNoteEditor = ( item ) => {
			setNoteTarget( item );
			noteDraft.value = item.noteText;
			dialogMode.value = item.note ? 'edit' : 'add';
			dialogOpen.value = true;
		};

		const openNoteRemover = ( item ) => {
			setNoteTarget( item );
			dialogMode.value = 'remove';
			dialogOpen.value = true;
		};

		const writeNote = async ( note ) => {
			noteBusy.value = true;
			noteError.value = '';
			try {
				await api.setErrorNote( noteTarget.value.message, note );
				dialogOpen.value = false;
				await populateLogs();
			} catch ( error ) {
				noteError.value = error.respJson?.detail?.message || error.message;
			}
			noteBusy.value = false;
		};

		const confirmDialog = () => writeNote( removing.value ? '' : noteDraft.value );

		onMounted( () => {
			intervalTimer = window.setInterval( populateLogs, INTERVAL );
			populateLogs();
		} );

		onUnmounted( () => {
			if ( intervalTimer ) {
				clearInterval( intervalTimer );
				intervalTimer = null;
			}
		} );

		return {
			headers,
			visibleItems,
			hideNoted,
			search,
			dialogOpen,
			dialogTitle,
			removing,
			confirmDialog,
			confirmDisabled,
			confirmLabel,
			noteDraft,
			noteError,
			noteTarget,
			openNoteEditor,
			openNoteRemover,
			cdxIconAdd,
			cdxIconEdit,
			cdxIconLinkExternal,
			cdxIconSearch,
			cdxIconTrash
		};
	}
} );
</script>

<style lang="less">
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';
@import ( reference ) '@wikimedia/codex/mixins/link.less';

.cdx-link {
	.cdx-mixin-link();

	.cdx-icon {
		color: inherit;
	}
}

.cdx-table {
	background-color: white;
}

.error-message {
	display: flex;
	align-items: flex-start;
	gap: @spacing-25;

	// This link holds an icon and no text, so it does not use the cdx-link
	// mixin. The mixin sizes the last icon of a link to match link text.
	&__search {
		flex-shrink: 0;
		color: @color-progressive;

		&:hover {
			color: @color-progressive--hover;
		}

		&:active {
			color: @color-progressive--active;
		}

		.cdx-icon {
			color: inherit;
		}
	}
}

.error-logs__filters {
	display: flex;
	align-items: center;
	gap: @spacing-100;

	.cdx-checkbox {
		flex-shrink: 0;
	}
}

.error-note {
	display: flex;
	align-items: flex-start;
	gap: @spacing-25;

	// A Codex icon-only button is 32px square, which dwarfs the note text.
	&__action.cdx-button {
		min-width: @spacing-150;
		min-height: @spacing-150;
		padding-right: @spacing-25;
		padding-left: @spacing-25;
	}

	&__dialog {
		&__subject {
			color: @color-subtle;
			font-size: @font-size-x-small;
			word-break: break-word;
		}

		&__byline {
			color: @color-subtle;
			font-size: @font-size-x-small;
		}

		&__quote {
			font-weight: @font-weight-bold;
			word-break: break-word;
		}
	}
}
</style>

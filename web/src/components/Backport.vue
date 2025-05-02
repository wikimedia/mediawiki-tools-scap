<template>
	<cdx-field id="backport" class="backport">
		<template #label>
			<h2 class="backport__heading">
				Backport Patches
			</h2>
		</template>

		<template #description>
			To run scap backport, enter one or more Gerrit change numbers or URLs
		</template>

		<div class="backport__input">
			<v-autocomplete
				ref="autocomplete"
				v-model="changeNumbers"
				:disabled="!idle"
				:hide-no-data="hideNoData"
				:items="menuItems"
				:loading="searching"
				aria-label="Search for one or more Gerrit patches"
				auto-select-first
				autofocus
				chips
				clear-on-select
				clearable
				closable-chips
				density="compact"
				menu-icon=""
				multiple
				no-filter
				persistent-clear
				persistent-placeholder
				placeholder="Enter change numbers or URLs"
				variant="outlined"
				@update:search="onSearch"
				@keydown.enter.stop.prevent="closeMenu"
			>
				<template #item="{ index, props }">
					<v-list-item
						v-bind="props"
						:key="index"
						role="option"
					/>
				</template>
				<template #no-data>
					<v-list-item>No results found.</v-list-item>
				</template>
				<template #append>
					<cdx-button
						:disabled="buttonDisabled"
						@click="startBackport"
					>
						Start Backport
					</cdx-button>
				</template>
			</v-autocomplete>
			<cdx-dialog
				v-model:open="alertDialogOpen"
				title="Failed to start backport"
				:use-close-button="alertDialogUseCloseButton"
			>
				{{ alertDialogText }}
			</cdx-dialog>
		</div>
	</cdx-field>
</template>

<script lang="ts">
import { ref, computed } from 'vue';
import { CdxButton, CdxDialog, CdxField } from '@wikimedia/codex';
import { VAutocomplete, VListItem } from 'vuetify/lib/components/index.mjs';
import useApi from '../api';
import { notificationsStore } from '@/state';

export default {
	name: 'SpBackport',
	components: {
		CdxButton,
		CdxDialog,
		CdxField,
		VAutocomplete,
		VListItem
	},

	props: {
		idle: {
			type: Boolean
		},
		initialChangeNumbers: {
			type: Array<string>,
			default: () => []
		}
	},

	setup( props ) {
		const autocomplete = ref( null );
		const api = useApi();
		const notifications = notificationsStore();
		const alertDialogOpen = ref( false );
		const alertDialogText = ref( '' );
		const alertDialogUseCloseButton = ref( true );
		const changeNumbers = ref( [] );
		const menuItems = ref( [] );
		const hideNoData = ref( true );
		const searching = ref( false );

		changeNumbers.value = props.initialChangeNumbers;

		const buttonDisabled = computed( () => {
			if ( changeNumbers.value.length > 0 ) {
				return false;
			} else {
				return true;
			}
		} );

		async function startBackport() {
			hideNoData.value = true;
			try {
				const res = await api.startBackport( changeNumbers.value );
				await notifications.setUserNotifications( res.id );
				changeNumbers.value = [];
			} catch ( error ) {
				alertDialogOpen.value = true;
				alertDialogText.value = error.respJson.detail.message || error.message;
			}
		}

		async function searchForChanges( changeId: string ) {
			searching.value = true;
			const data = await api.searchPatch( changeId );
			const results = data.map( ( item ) => {
				// eslint-disable-next-line no-underscore-dangle
				const changeNumber = String( item._number );
				return {
					title: changeNumber, // Gerrit change number, ex. 1084887
					props: {
						subtitle: item.subject // The commit subject line
					},
					value: changeNumber // Gerrit change number, ex. 1084887
				};
			} );
			searching.value = false;
			return results;
		}

		async function onSearch( value: string ) {
			if ( !value ) {
				hideNoData.value = true;
				menuItems.value = [];
				return;
			}
			hideNoData.value = false;
			const results = await searchForChanges( value );
			menuItems.value = results;
		}

		async function closeMenu() {
			autocomplete.value.menu = false;
		}

		return {
			autocomplete,
			alertDialogOpen,
			alertDialogText,
			alertDialogUseCloseButton,
			buttonDisabled,
			changeNumbers,
			closeMenu,
			hideNoData,
			menuItems,
			onSearch,
			searching,
			startBackport
		};
	}
};
</script>

<style lang="less">
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

.backport {
	&.cdx-field {
		margin-bottom: @spacing-150;
	}

	&__heading {
		font-size: @font-size-x-large;
	}

	&__input {
		display: flex;

		.v-input__append {
			margin-inline-start: 0;

			.cdx-button {
				border-top-left-radius: 0px;
				border-bottom-left-radius: 0px;
				min-height: 40px;
			}
		}
	}
}

</style>

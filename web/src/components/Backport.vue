<template>
	<cdx-field id="backport" class="backport">
		<template #label>
			<h2 class="backport__heading">
				Backport Patches
			</h2>
		</template>

		<template #description>
			To run scap backport, enter one or more Gerrit change numbers
			separated by spaces.
		</template>

		<div class="backport__input">
			<v-autocomplete
				ref="$autocomplete"
				v-model="changeNumbers"
				:disabled="!idle"
				:hide-no-data="searching"
				:items="menuItems"
				:loading="searching"
				aria-label="Search for one or more Gerrit patches"
				auto-select-first="exact"
				autofocus
				chips
				clear-on-select
				clearable
				closable-chips
				density="compact"
				multiple
				no-filter
				persistent-clear
				persistent-placeholder
				placeholder="Enter change numbers"
				variant="outlined"
				@update:search="onSearch"
			>
				<template #item="{ index, props, item }">
					<v-list-item
						v-bind="props"
						:key="index"
						role="option"
					>
						{{ item.text }}
					</v-list-item>
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
		</div>
	</cdx-field>
</template>

<script lang="ts">
import { ref, computed } from 'vue';
import { CdxButton, CdxField } from '@wikimedia/codex';
import { VAutocomplete, VListItem } from 'vuetify/lib/components/index.mjs';
import useApi from '../api';

export default {
	name: 'SpBackport',
	components: {
		CdxButton,
		CdxField,
		VAutocomplete,
		VListItem
	},

	props: {
		idle: {
			type: Boolean
		}
	},

	setup() {
		const $autocomplete = ref( null );
		const api = useApi();
		const changeNumbers = ref( [] );
		const menuItems = ref( [] );
		const searching = ref( false );
		const RE_DELIMITERS = /(?:,| )+/;

		const buttonDisabled = computed( () => {
			if ( changeNumbers.value.length > 0 ) {
				return false;
			} else {
				return true;
			}
		} );

		async function startBackport() {
			await api.startBackport(
				changeNumbers.value.map( ( id ) => id.value )
			);
			changeNumbers.value = [];
		}

		async function searchForChanges( value ) {
			searching.value = true;
			// Search by change number only returns exact matches
			const data = await api.searchPatch( `change:"${ value }"`, 1 );
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

		async function onSearch( value ) {
			if ( !value ) {
				menuItems.value = [];
				return;
			}

			// Simulate VCombobox's delimiters function
			const values = value.split( RE_DELIMITERS );
			if ( values.length > 1 ) {
				for ( let val of values ) {
					val = val.trim();
					if ( val ) {
						const found = await searchForChanges( val );
						if ( found.length === 1 && found[ 0 ].value === val ) {
							changeNumbers.value.push( found[ 0 ] );
						}
					}
				}
				// Blank search-input by blurring the widget
				$autocomplete.value.blur();
				return;
			}

			const results = await searchForChanges( value );
			menuItems.value = results;
		}

		return {
			$autocomplete,
			buttonDisabled,
			changeNumbers,
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

		&__multiselect {
			flex: 1 0 auto;
		}

		.cdx-chip-input {
			border-top-right-radius: 0px;
			border-bottom-right-radius: 0px;
		}

		.cdx-button {
			border-top-left-radius: 0px;
			border-bottom-left-radius: 0px;
		}
	}
}

</style>

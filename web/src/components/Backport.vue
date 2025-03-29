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
			<v-combobox
				v-model="changeNumbers"
				:delimiters="[ ',', ' ' ]"
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
			</v-combobox>
		</div>
	</cdx-field>
</template>

<script lang="ts">
import { ref, computed } from 'vue';
import { CdxButton, CdxField } from '@wikimedia/codex';
import { VCombobox, VListItem } from 'vuetify/lib/components/index.mjs';
import useApi from '../api';

export default {
	name: 'SpBackport',
	components: {
		CdxButton,
		CdxField,
		VCombobox,
		VListItem
	},

	props: {
		idle: {
			type: Boolean
		}
	},

	setup() {
		const api = useApi();
		const changeNumbers = ref( [] );
		const menuItems = ref( [] );
		const searching = ref( false );

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

		async function fetchResults( changeId ) {
			searching.value = true;
			const data = await api.searchPatch(
				`change:"${ changeId }"`,
				10
			);
			searching.value = false;
			return data;
		}

		async function onSearch( value ) {
			if ( !value ) {
				menuItems.value = [];
				return;
			}
			const data = await fetchResults( value );
			const results = data.map( ( item ) => {
				// eslint-disable-next-line no-underscore-dangle
				const changeNumber = String( item._number );

				return {
					title: changeNumber, // Gerrit patch number, ex. 1084887
					props: {
						subtitle: item.subject // The commit subject line
					},
					value: changeNumber // Gerrit patch number, ex. 1084887
				};
			} );
			menuItems.value = results;
		}

		return {
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

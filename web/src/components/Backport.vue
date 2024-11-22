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
			<cdx-multiselect-lookup
				v-model:input-chips="changeIds"
				v-model:selected="selection"
				class="backport__input__multiselect"
				:disabled="!idle"
				:menu-items="menuItems"
				:menu-config="menuConfig"
				placeholder="Enter change numbers"
				aria-label="Search for one or more Gerrit patches"
				@input="onInput"
			>
				<template #no-results>
					No results found.
				</template>
			</cdx-multiselect-lookup>

			<cdx-button
				:disabled="buttonDisabled"
				@click="startBackport"
			>
				Start Backport
			</cdx-button>
		</div>
	</cdx-field>
</template>

<script lang="ts">
import { ref, computed } from 'vue';
import { CdxButton, CdxField, CdxMultiselectLookup } from '@wikimedia/codex';
import useApi from '../api';

export default {
	name: 'SpBackport',
	components: {
		CdxButton,
		CdxField,
		CdxMultiselectLookup
	},

	props: {
		idle: {
			type: Boolean
		}
	},

	setup() {
		const api = useApi();
		const changeIds = ref( [] );
		const selection = ref( [] );
		const menuItems = ref( [] );

		const menuConfig = {
			boldLabel: true,
			visibleItemLimit: 10
		};

		const buttonDisabled = computed( () => {
			if ( changeIds.value.length > 0 ) {
				return false;
			} else {
				return true;
			}
		} );

		async function startBackport() {
			await api.startBackport( changeIds.value.map( ( id ) => id.value ) );
			changeIds.value = [];
		}

		async function fetchResults( changeId ) {
			return await api.searchPatch(
				`change:"${ changeId }"`,
				10
			);
		}

		async function onInput( value ) {
			if ( !value ) {
				menuItems.value = [];
				return;
			}

			const data = await fetchResults( value );

			const results = data.map( ( item ) => {
				// eslint-disable-next-line no-underscore-dangle
				const changeNumber = String( item._number );

				return {
					label: changeNumber, // the Gerrit patch number, ex. 1084887
					description: item.subject, // The commit subject line
					value: changeNumber // the Gerrit patch number, ex. 1084887
				};
			} );

			menuItems.value = results;
		}

		return {
			buttonDisabled,
			changeIds,
			selection,
			menuItems,
			menuConfig,
			startBackport,
			onInput
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
		font-size: 1.25rem;
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

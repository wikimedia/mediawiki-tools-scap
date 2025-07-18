<template>
	<cdx-table
		caption="MediaWiki Error Logs"
		:hide-caption="true"
		:columns="columns"
		:data="data"
		:show-vertical-borders="true"
	>
		<template #item-message="{ item }">
			<a
				:href="item.link"
				class="cdx-link"
				target="_blank"
			>
				{{ item.message }}
				<cdx-icon :icon="cdxIconLinkExternal" />
			</a>
		</template>
	</cdx-table>
</template>

<script>
import { onMounted, onUnmounted, defineComponent, ref } from 'vue';
import { CdxIcon, CdxTable } from '@wikimedia/codex';
import { cdxIconLinkExternal } from '@wikimedia/codex-icons';

import useApi from '../api';

export default defineComponent( {
	name: 'SpLogs',
	components: {
		CdxIcon,
		CdxTable
	},
	setup() {
		const columns = [
			{ id: 'count', label: 'Count', textAlign: 'number' },
			{ id: 'versions', label: 'Versions' },
			{ id: 'message', label: 'Message' }
		];
		const data = ref();
		const total = ref();
		const api = useApi();
		const maxMessageLength = 200;
		const INTERVAL = 15000;
		let intervalTimer = null;

		const createOpenSearchLink = ( errorMessage, fieldName = 'normalized_message', timeRange = '24h' ) => {
			const dashboard = 'https://logstash.wikimedia.org/app/dashboards#/view/mediawiki-errors';
			const timeFrom = `now-${ timeRange }`;
			const errorMessageForURI = encodeURIComponent( errorMessage );

			const filterStr = `(meta:(alias:!n,disabled:!f,key:${ fieldName },negate:!f,` +
				`params:(query:'${ errorMessageForURI }'),type:phrase),` +
				`query:(match_phrase:(${ fieldName }:'${ errorMessageForURI }')))`;

			const globalState = `(filters:!(),refreshInterval:(pause:!t,value:0),time:(from:${ timeFrom },to:now))`;

			const appState = `(columns:!(_source),filters:!(${ filterStr }),index:'logstash-*',` +
				'interval:auto,query:(language:kuery,query:\'\'),sort:!())';

			return `${ dashboard }?_g=${ globalState }&_a=${ appState }`;
		};

		const populateLogs = async () => {
			const resp = await api.getLogs(),
				newData = [];
			let sum = 0;
			for ( const [ key, value ] of Object.entries( resp.log ) ) {
				let message = key;
				const link = createOpenSearchLink( key );
				if ( key.length > maxMessageLength ) {
					message = key.slice( 0, maxMessageLength ) + '...';
				}
				newData.push( {
					count: value.count,
					versions: value.versions.join( ' ' ),
					message: { message: message, link: link }
				} );
				sum += value.count;
			}

			newData.sort( ( a, b ) => {
				if ( a.count < b.count ) {
					return 1;
				} else if ( a.count > b.count ) {
					return -1;
				}
				return 0;
			} );
			total.value = sum;
			data.value = newData;
		};

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
			columns,
			data,
			cdxIconLinkExternal
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
</style>

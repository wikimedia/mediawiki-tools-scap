<template>
    <div>
        <nav class="spiderpig-nav">
            <div class="spiderpig-nav__start">
                <img src="./assets/spiderpig.png" />
            </div>
            <div class="spiderpig-nav__center">
                <template v-if="$route.params.job_id && $route.meta.title">
                    <h1>{{ $route.meta.title }}{{ $route.params.job_id }}</h1>
                </template>
                <h1 v-else>{{ $route.meta.title }}</h1>
            </div>
            <div class="spiderpig-nav__end">
                <CdxButton v-if="api && api.isAuthenticated" @click="logout">Logout</CdxButton>
            </div>
        </nav>

        <main>
            <RouterView />
        </main>
    </div>
</template>

<script>
import '@wikimedia/codex/dist/codex.style.css'

import useApi from './api';

import { CdxButton } from '@wikimedia/codex';

export default {
    data() {
        return {
            api: null
        }
    },
    components: {
        CdxButton,
    },
    mounted() {
        this.api = useApi()
    },
    methods: {
        logout() {
            this.api.logout()
        }
    }

}
</script>

<style scoped>
.spiderpig-nav {
    display: flex;

}

.spiderpig-nav__start {
    flex: 0 0 3rems;
}
.spiderpig-nav__center {
    flex: 1 1 auto;
}

.spiderpig-nav__end {
    flex: 0 0 100px;
}
</style>
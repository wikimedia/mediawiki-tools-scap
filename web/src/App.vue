<template>
    <div>
        <div id="spiderpig-banner">
            <img src="./assets/spiderpig.png"/>
            <h1>Welcome to SpiderPig</h1>
        </div>
        <button v-if="isAuthenticated()" @click="logout">Logout</button>
        <main>
            <RouterView/>
        </main>
    </div>
</template>

<script setup>
import { isAuthenticated, logout } from './common';

</script>

<script>
export default {
    methods: {
        logout() {
            logout()
        }
    }

}
</script>

<style scoped>
#spiderpig-banner {
    display: flex;
    gap: 10px;
    padding: 5px;
}
</style>
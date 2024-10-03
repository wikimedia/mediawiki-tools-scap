<template>
    <div>
        <div id="spiderpig-banner">
            <img src="./assets/spiderpig.png"/>
            <h1>Welcome to SpiderPig</h1>
        </div>
        <main>
            <RouterView/>
        </main>
    </div>
</template>

<script>
export default {

}
</script>

<style scoped>
#spiderpig-banner {
    display: flex;
    gap: 10px;
    padding: 5px;
}
</style>
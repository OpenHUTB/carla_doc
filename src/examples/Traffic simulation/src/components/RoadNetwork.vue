<template>
    <div class="home">
      <div class="main">
        <div class="Simulation"><Simulation /></div>
        <div class="PredictionComparison"><PredictionComparison /></div>
      </div>
    </div>
</template>
  
  <style>
  img {
    width: 100%;
    height: 100%;
    object-fit: cover;
  }

  .main {
    display: flex;
    flex-direction: column;
  }
  </style>
  
  <script>
  import Simulation from "./RightComponent/Simulation.vue";
  import PredictionComparison from "./RightComponent/PredictionComparison.vue";
  export default {
    components: {
      Simulation,
      PredictionComparison
    },
    methods: {},
  };
  </script>
  
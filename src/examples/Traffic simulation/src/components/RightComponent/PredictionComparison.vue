<template>
    <div class="home">
      <div class="main">
        <div class="Index"><Index /></div>
        <div class="Trend"><Trend /></div>
        <div class="Comparison"><Comparison /></div>
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
  import Index from "./PredictionComparison/Index.vue"
  import Trend from "./PredictionComparison/Trend.vue"
  import Comparison from "./PredictionComparison/Comparison.vue"

  export default {
    components: {
        Index,
        Trend,
        Comparison
    },
    methods: {},
  };
  </script>
  
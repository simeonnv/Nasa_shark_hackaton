import "./App.css";
import MapGL from "./Map";

function App() {
  return (
    <div className="w-screen h-screen">
      <div className="absolute z-10 bg-blue-500 p-2 text-white border border-black rounded-xl m-2 opacity-90 text-center">
        <p className="opacity-100">Amount of sharks in the simulation: 300</p>
        <p className="opacity-100">Speed: 10 tps</p>
      </div>
      <MapGL />
    </div>
  );
}

export default App;

import React, { useEffect, useRef } from "react";
import {
  Map as GLMap,
  TileLayer,
  Marker,
  VectorLayer,
  Coordinate,
} from "maptalks-gl";
import "maptalks-gl/dist/maptalks-gl.css";

interface SharkData {
  position: { x: number; y: number };
  rotation_rad: number;
  speed: number;
}

interface MapGLProps {
  center?: Coordinate;
  zoom?: number;
  style?: React.CSSProperties;
}

const MapGL: React.FC<MapGLProps> = ({
  center = { x: 0, y: 0, z: 0 } as Coordinate,
  zoom = 2,
  style = { width: "100%", height: "100%" },
}) => {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const mapRef = useRef<GLMap | null>(null);
  const vectorLayerRef = useRef<VectorLayer | null>(null);
  const sharkMarkersRef = useRef<Record<number, Marker>>({}); // indexed by shark id

  useEffect(() => {
    if (!containerRef.current) return;

    // 1️⃣ Initialize map
    mapRef.current = new GLMap(containerRef.current, {
      center,
      zoom,
      pitch: 0,
      baseLayer: new TileLayer("base", {
        urlTemplate:
          "https://tiles.stadiamaps.com/tiles/stamen_watercolor/{z}/{x}/{y}.jpg",
        subdomains: ["a", "b", "c"],
        attribution: "&copy; OpenStreetMap contributors",
      }),
    });

    // 2️⃣ Vector layer for sharks
    vectorLayerRef.current = new VectorLayer("sharks").addTo(mapRef.current);

    // 3️⃣ WebSocket connection
    const ws = new WebSocket("ws://localhost:25555");

    ws.onopen = () => {
      console.log("✅ WebSocket connected to ws://localhost:2555");
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data) as { sharks: SharkData[] };
        if (!data.sharks || !Array.isArray(data.sharks)) return;

        data.sharks.forEach((shark, idx) => {
          const { position, rotation_rad } = shark;
          const { x, y } = position;

          // if marker exists, update it
          if (sharkMarkersRef.current[idx]) {
            sharkMarkersRef.current[idx].setCoordinates([x, y]);
            sharkMarkersRef.current[idx].updateSymbol({
              markerRotation: (rotation_rad * 180) / Math.PI, // convert rad→deg
            });
          } else {
            // else create new marker
            const marker = new Marker([x, y], {
              symbol: {
                markerFile:
                  "https://cdn-icons-png.flaticon.com/512/616/616408.png", // shark icon
                markerWidth: 10,
                markerHeight: 10,
                markerRotation: (rotation_rad * 180) / Math.PI,
              },
            });
            sharkMarkersRef.current[idx] = marker;
            vectorLayerRef.current?.addGeometry(marker);
          }
        });
      } catch (err) {
        console.error("❌ Failed to parse WS message:", err);
      }
    };

    ws.onclose = () => {
      console.log("⚠️ WebSocket disconnected");
    };

    // Cleanup
    return () => {
      ws.close();
      mapRef.current?.remove();
    };
  }, []);

  return <div ref={containerRef} style={style} />;
};

export default MapGL;

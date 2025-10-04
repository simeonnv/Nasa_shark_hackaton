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
  const sharkMarkersRef = useRef<Record<number, Marker>>({});

  // Calculate marker size based on zoom level
  const getMarkerSize = (zoomLevel: number) => {
    const baseSize = 10;
    const scaleFactor = Math.pow(1.3, zoomLevel - 2); // Exponential scaling
    return baseSize * scaleFactor;
  };

  // Update all shark markers with new size
  const updateAllMarkerSizes = () => {
    if (!mapRef.current) return;
    const currentZoom = mapRef.current.getZoom();
    const size = getMarkerSize(currentZoom);

    Object.values(sharkMarkersRef.current).forEach((marker) => {
      marker.updateSymbol({
        markerWidth: size,
        markerHeight: size,
      });
    });
  };

  useEffect(() => {
    if (!containerRef.current) return;

    // Initialize map
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

    vectorLayerRef.current = new VectorLayer("sharks").addTo(mapRef.current);

    // Listen to zoom changes
    mapRef.current.on("zoomend", updateAllMarkerSizes);

    // WebSocket connection
    const ws = new WebSocket("ws://localhost:25555");

    ws.onopen = () => {
      console.log("✅ WebSocket connected to ws://localhost:25555");
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data) as { sharks: SharkData[] };
        if (!data.sharks || !Array.isArray(data.sharks)) return;

        const currentZoom = mapRef.current?.getZoom() || zoom;
        const size = getMarkerSize(currentZoom);

        data.sharks.forEach((shark, idx) => {
          const { position, rotation_rad } = shark;
          const { x, y } = position;

          if (sharkMarkersRef.current[idx]) {
            // Update existing marker
            sharkMarkersRef.current[idx].setCoordinates([x, y]);
            sharkMarkersRef.current[idx].updateSymbol({
              markerRotation: (rotation_rad * 180) / Math.PI,
            });
          } else {
            // Create new marker with current zoom size
            const marker = new Marker([x, y], {
              symbol: {
                markerFile:
                  "https://cdn-icons-png.flaticon.com/512/9339/9339269.png",
                markerWidth: size,
                markerHeight: size,
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

# Flock-Surveilence-Scanner

Uses Flock You and Deflock combined with two ESP-32 Devs and a GPS to preemptively alert to known Flock or other ALPRs, and Wi-Fi/BLE to detect new/unknown units.

To get accurate up-to-date Flock Camera locations usable with the GPS:

1. Go to [https://overpass-turbo.eu/](https://overpass-turbo.eu/)
2. Paste the following script and change the state:
    ```cpp
    [out:json][timeout:60];

    // Define the search area as the state of New Jersey
    area["ISO3166-2"="US-NJ"]->.searchArea;

    (
      node["man_made"="surveillance"]["surveillance:type"="ALPR"](area.searchArea);
      node["manufacturer"="Flock Safety"](area.searchArea);
    );

    out body;
    >;
    out skel qt;
    ```
3. Download the file and use the python script to turn it into `cameras.bin`.

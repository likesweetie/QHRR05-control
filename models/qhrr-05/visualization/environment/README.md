# Default product-studio environment

`product_studio_ibl.ktx` is generated locally from an analytic neutral environment with three feathered rectangular softboxes and dark negative fill. It contains no photographed window/furniture details or narrow stripes. It is the default IBL in `scripts/visualization_process.sh`.

```bash
python3 tools/prepare_product_studio_ibl.py
```

This command uses the pinned Filament `cmgen` already in the visualization build and does not access the network. It writes a 512px GGX-prefiltered cubemap with diffuse irradiance SH. `product_studio_ibl.json` records source/output SHA256 and source dimensions. The runtime KTX is included; normal startup never regenerates it and fails explicitly if it is missing. This generated asset is authored within this repository; no external image license is required.

The original model geometry is unchanged. Brightness is configured by `filament.fallback.environment_light_intensity` in `scene_pbr.xml`.

# Optional Studio Small 09 environment

- Author: Sergej Majboroda / Poly Haven.
- Source: [Studio Small 09](https://polyhaven.com/a/studio_small_09).
- License: [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/).
- [Poly Haven asset licensing](https://polyhaven.com/license) permits use and redistribution.
- Original file: `https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/1k/studio_small_09_1k.hdr`.
- Original SHA256: `e7cfda5f4e98e623db12b8bfd0184e048488e4855d9c83e2751fb44a32e80c45`.

The included `studio_small_09_ibl.ktx` is a 256-pixel cubemap with GGX roughness
mip levels and diffuse irradiance spherical harmonics. It was prefiltered with
MuJoCo 3.16.0's pinned Filament rc/1.77.3 `cmgen`:

```bash
python3 tools/prepare_studio_ibl.py
```

The command explicitly downloads the pinned HDR source into ignored
`build-filament/environment-source/` and regenerates this runtime KTX.
Normal application startup uses the included KTX offline and never downloads
or substitutes another environment when this file is missing.

Initial generated KTX SHA256:
`f8f027ae5ec668ecf00f3af1ae762ab104dc37bcf29da33b27efdb5bfe74bdab`.
Prefilter floating-point output can differ between toolchains/platforms; the
original HDR source hash is checked before conversion.

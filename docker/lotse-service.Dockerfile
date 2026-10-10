# Builds and packages the Lotse service. The builder stage reuses the
# development image, so the C++ toolchain and ROS are not installed twice. The
# runtime stage is a small Debian carrying only libstdc++ and the service files.

ARG LOTSE_DEV_IMAGE=lotse-dev:jazzy
FROM ${LOTSE_DEV_IMAGE} AS builder

WORKDIR /src
COPY . .
RUN cmake -S . -B /build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_COMPILER=clang++ \
        -DLOTSE_BUILD_TESTS=OFF \
 && cmake --build /build --target lotse_service_app -j "$(nproc)"

FROM ubuntu:24.04 AS runtime

# Ubuntu 24.04 matches the development image, so the binary's glibc and
# libstdc++ are present; SQLite is linked in statically.

WORKDIR /lotse
COPY --from=builder /build/service/lotse_service_app /lotse/lotse_service
COPY maps /lotse/maps
COPY config /lotse/config
COPY service/dashboard /lotse/service/dashboard

RUN mkdir -p /data
EXPOSE 8080

# The container listens on all interfaces so Docker can forward the port; the
# published port itself is bound to 127.0.0.1 on the host by docker-compose.
ENTRYPOINT ["/lotse/lotse_service", "--config", "/lotse/config/lotse.conf", "--bind", "0.0.0.0"]

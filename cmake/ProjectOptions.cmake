# Per-target warnings (as errors) and sanitizers.

function(fusion_configure_target target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive- /WX)
  else()
    target_compile_options(${target} PRIVATE
      -Wall -Wextra -Wpedantic
      -Wshadow                # local hides another variable
      -Wconversion            # implicit narrowing, e.g. double -> int
      -Wsign-conversion       # int <-> size_t surprises
      -Wold-style-cast        # use static_cast<> etc.
      -Wnon-virtual-dtor      # polymorphic base without virtual dtor
      -Woverloaded-virtual
      -Wnull-dereference
      -Wdouble-promotion      # float silently promoted to double
      -Werror
    )
  endif()

  if(FUSION_ENABLE_SANITIZERS AND NOT MSVC)
    target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=address,undefined)
  endif()
endfunction()
